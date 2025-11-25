using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Shapes;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Threading;
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Numerics;

namespace StewartPlatformPi
{
    public partial class MainWindow : Window
    {
        // --- 1. CORE LOGIC ---
        StewartPlatform platform = new StewartPlatform();
        SerialPort? arduinoPort;
        DispatcherTimer sendTimer;
        DispatcherTimer resetAnimationTimer;

        // --- 2. VISUAL OBJECTS ---
        Polygon basePoly = new Polygon() { Stroke = Brushes.LightGreen, StrokeThickness = 3 };
        Polygon platPoly = new Polygon() { Stroke = Brushes.LightBlue, StrokeThickness = 3 };
        Line[] hornLines = new Line[6];
        Line[] rodLines = new Line[6];

        // Screen Center logic
        double centerX = 300;
        double centerY = 300;
        double scale = 1.8;

        public MainWindow()
        {
            InitializeComponent();

            // Init Visuals on Canvas
            if (ViewCanvas != null)
            {
                ViewCanvas.Children.Add(basePoly);
                ViewCanvas.Children.Add(platPoly);

                for (int i = 0; i < 6; i++)
                {
                    hornLines[i] = new Line() { Stroke = Brushes.Red, StrokeThickness = 4 };
                    rodLines[i] = new Line() { Stroke = Brushes.White, StrokeThickness = 2 };

                    ViewCanvas.Children.Add(hornLines[i]);
                    ViewCanvas.Children.Add(rodLines[i]);
                }
            }

            // SIMPLIFIED RESIZE LOGIC (No complex Observables)
            this.SizeChanged += (s, e) =>
            {
                if (ViewCanvas != null && ViewCanvas.Bounds.Width > 0)
                {
                    centerX = ViewCanvas.Bounds.Width / 2;
                    centerY = ViewCanvas.Bounds.Height / 2 + 80;
                    UpdateDraw();
                }
            };

            LoadAvailablePorts();

            // Setup Timers
            sendTimer = new DispatcherTimer();
            sendTimer.Interval = TimeSpan.FromMilliseconds(40);
            sendTimer.Tick += SendDataToArduino;

            resetAnimationTimer = new DispatcherTimer();
            resetAnimationTimer.Interval = TimeSpan.FromMilliseconds(20);
            resetAnimationTimer.Tick += AnimateResetStep;

            // Initial Calculation
            platform.ApplyTranslationAndRotation(0, 0, 0, 0, 0, 0);
            UpdateDraw();
        }

        // --- 3. MATH: PROJECT 3D ROBOT TO 2D SCREEN ---
        private Point Project(Vector3 v)
        {
            double isoAngle = Math.PI / 6;
            double screenX = centerX + (v.X * Math.Cos(isoAngle) + v.Y * Math.Cos(isoAngle)) * scale;
            double screenY = centerY - (v.Z * scale) + (v.X * Math.Sin(isoAngle) - v.Y * Math.Sin(isoAngle)) * scale * 0.5;
            return new Point(screenX, screenY);
        }

        private void UpdateDraw()
        {
            if (ViewCanvas == null || ViewCanvas.Bounds.Width <= 0) return;

            var basePoints = new List<Point>();
            foreach (var p in platform.BasePoints) basePoints.Add(Project(p));
            basePoly.Points = basePoints;

            var platPoints = new List<Point>();
            foreach (var p in platform.PlatformPoints) platPoints.Add(Project(p));
            platPoly.Points = platPoints;

            for (int i = 0; i < 6; i++)
            {
                Point b = Project(platform.BasePoints[i]);
                Point h = Project(platform.HornEndPoints[i]);
                Point p = Project(platform.PlatformPoints[i]);

                hornLines[i].StartPoint = b;
                hornLines[i].EndPoint = h;

                rodLines[i].StartPoint = h;
                rodLines[i].EndPoint = p;
            }
        }

        // --- 4. EVENTS ---
        private void Slider_ValueChanged(object? sender, AvaloniaPropertyChangedEventArgs e)
        {
            if (e.Property.Name != "Value") return;
            if (platform == null || SldPosX == null) return;

            double rx = SldRotX.Value * (Math.PI / 180.0);
            double ry = SldRotY.Value * (Math.PI / 180.0);
            double rz = SldRotZ.Value * (Math.PI / 180.0);

            platform.ApplyTranslationAndRotation(SldPosX.Value, SldPosY.Value, SldPosZ.Value, rx, ry, rz);
            UpdateUI();
            UpdateDraw();
        }

        private void UpdateUI()
        {
            if (TxtServo0 == null) return;
            TxtServo0.Text = $"S0: {platform.GetAlphaDegree(0):F2}°";
            TxtServo1.Text = $"S1: {platform.GetAlphaDegree(1):F2}°";
            TxtServo2.Text = $"S2: {platform.GetAlphaDegree(2):F2}°";
            TxtServo3.Text = $"S3: {platform.GetAlphaDegree(3):F2}°";
            TxtServo4.Text = $"S4: {platform.GetAlphaDegree(4):F2}°";
            TxtServo5.Text = $"S5: {platform.GetAlphaDegree(5):F2}°";
        }

        private void BtnConnect_Click(object? sender, RoutedEventArgs e)
        {
            if (arduinoPort != null && arduinoPort.IsOpen)
            {
                try { arduinoPort.Close(); } catch { }
                sendTimer.Stop();
                BtnConnect.Content = "Connect";
                TxtStatus.Text = "Disconnected";
                TxtStatus.Foreground = Brushes.Red;
            }
            else
            {
                if (PortSelector.SelectedItem == null) return;
                try
                {
                    arduinoPort = new SerialPort(PortSelector.SelectedItem.ToString()!, 115200);
                    arduinoPort.Open();
                    sendTimer.Start();
                    BtnConnect.Content = "Disconnect";
                    TxtStatus.Text = "Connected";
                    TxtStatus.Foreground = Brushes.Green;
                }
                catch (Exception ex)
                {
                    var msgBox = new Window { Width = 300, Height = 100, Content = new TextBlock { Text = "Error: " + ex.Message, Margin = Thickness.Parse("10") } };
                    msgBox.Show();
                }
            }
        }

        private void LoadAvailablePorts()
        {
            string[] ports = SerialPort.GetPortNames();
            PortSelector.ItemsSource = ports;
            if (ports.Length > 0) PortSelector.SelectedIndex = 0;
        }

        private void BtnReset_Click(object? sender, RoutedEventArgs e)
        {
            resetAnimationTimer.Start();
        }

        private void AnimateResetStep(object? sender, EventArgs e)
        {
            double step = 1.0;
            bool MoveSlider(Slider sld)
            {
                if (sld == null) return true;
                if (Math.Abs(sld.Value) > 0.1)
                {
                    if (sld.Value > 0) sld.Value -= step; else sld.Value += step;
                    if (Math.Abs(sld.Value) < step) sld.Value = 0;
                    return false;
                }
                sld.Value = 0;
                return true;
            }

            bool done = true;
            if (!MoveSlider(SldPosX)) done = false;
            if (!MoveSlider(SldPosY)) done = false;
            if (!MoveSlider(SldPosZ)) done = false;
            if (!MoveSlider(SldRotX)) done = false;
            if (!MoveSlider(SldRotY)) done = false;
            if (!MoveSlider(SldRotZ)) done = false;

            if (done) resetAnimationTimer.Stop();
        }

        private void SendDataToArduino(object? sender, EventArgs e)
        {
            if (arduinoPort == null || !arduinoPort.IsOpen) return;
            try
            {
                byte[] header = { 0x6A, 0x6A };
                arduinoPort.Write(header, 0, 2);
                string data = "";
                for (int i = 0; i < 6; i++)
                {
                    int val = (int)(platform.GetAlphaDegree(i) * 100);
                    data += val.ToString();
                    if (i < 5) data += ",";
                }
                data += "\n";
                arduinoPort.Write(data);
            }
            catch { }
        }
    }
}