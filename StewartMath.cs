using System;
using System.Numerics;

namespace StewartPlatformPi
{
    public class StewartPlatform
    {
        // --- PHYSICAL CONSTANTS (Updated from details.pdf) ---

        // BASE ANGLES (Uniformly spaced at 0, 60, 120...) 
        private readonly double[] BASE_ANGLES = {
            0, 60, 120, 180, 240, 300
        };

        // PLATFORM ANGLES (Paired sectors) 
        // Mapped to the closest Base Angle:
        // Servo 0 (0°)   -> Platform 345° (-15°)
        // Servo 1 (60°)  -> Platform 75°
        // Servo 2 (120°) -> Platform 105°
        // Servo 3 (180°) -> Platform 195°
        // Servo 4 (240°) -> Platform 225°
        // Servo 5 (300°) -> Platform 315°
        private readonly double[] PLATFORM_ANGLES = {
            345, 75, 105, 195, 225, 315
        };

        // BETA ANGLES: The orientation of the servo arm rotation plane.
        // We assume each servo arm rotates perpendicular to the radius (Tangent).
        // The Arduino code handles the +/- direction flip for odd/even servos.
        private readonly double[] BETA = {
            Math.PI / 2,        // 0° + 90°
            5 * Math.PI / 6,    // 60° + 90° (150°)
            7 * Math.PI / 6,    // 120° + 90° (210°)
            3 * Math.PI / 2,    // 180° + 90° (270°)
            11 * Math.PI / 6,   // 240° + 90° (330°)
            Math.PI / 6         // 300° + 90° (30°)
        };

        // Dimensions (mm)
        public const float BASE_RADIUS = 86f;      // 
        public const float PLATFORM_RADIUS = 50f;  // 
        public const float HORN_LENGTH = 27.845f;  // 
        public const float ROD_LENGTH = 110f;      // 

        // Calculated Initial Height (approximate home position)
        // h0 = Sqrt(Rod^2 - (BaseRad - PlatRad)^2)
        // h0 = Sqrt(110^2 - (86 - 50)^2) = ~104mm
        private const float INITIAL_HEIGHT = 104.0f;

        // 3D DRAWING POINTS
        public Vector3[] BasePoints { get; private set; } = new Vector3[6];
        public Vector3[] PlatformPoints { get; private set; } = new Vector3[6];
        public Vector3[] HornEndPoints { get; private set; } = new Vector3[6];

        // Internal Math Vectors
        private Vector3[] b = new Vector3[6];
        private Vector3[] p = new Vector3[6];

        // Servo Angles
        public double[] Alpha { get; private set; } = new double[6];

        // Current State
        private Vector3 Translation;
        private Vector3 Rotation;

        public StewartPlatform()
        {
            // Initialize Geometry
            for (int i = 0; i < 6; i++)
            {
                // Calculate Base Points (b)
                float xb = BASE_RADIUS * (float)Math.Cos(ToRadians(BASE_ANGLES[i]));
                float yb = BASE_RADIUS * (float)Math.Sin(ToRadians(BASE_ANGLES[i]));
                b[i] = new Vector3(xb, yb, 0);
                BasePoints[i] = b[i];

                // Calculate Platform Points (p)
                float px = PLATFORM_RADIUS * (float)Math.Cos(ToRadians(PLATFORM_ANGLES[i]));
                float py = PLATFORM_RADIUS * (float)Math.Sin(ToRadians(PLATFORM_ANGLES[i]));
                p[i] = new Vector3(px, py, 0);
            }
        }

        public void ApplyTranslationAndRotation(double x, double y, double z, double rotX, double rotY, double rotZ)
        {
            Translation = new Vector3((float)x, (float)y, (float)z);
            Rotation = new Vector3((float)rotX, (float)rotY, (float)rotZ);
            CalculateAngles();
        }

        private void CalculateAngles()
        {
            Vector3 h0 = new Vector3(0, 0, INITIAL_HEIGHT);

            for (int i = 0; i < 6; i++)
            {
                // 1. Calculate q[i] (Platform Joint World Position)
                float cx = (float)Math.Cos(Rotation.X); float sx = (float)Math.Sin(Rotation.X);
                float cy = (float)Math.Cos(Rotation.Y); float sy = (float)Math.Sin(Rotation.Y);
                float cz = (float)Math.Cos(Rotation.Z); float sz = (float)Math.Sin(Rotation.Z);

                // Rotation Matrix
                float qx = (cz * cy) * p[i].X + (-sz * cx + cz * sy * sx) * p[i].Y + (sz * sx + cz * sy * cx) * p[i].Z;
                float qy = (sz * cy) * p[i].X + (cz * cx + sz * sy * sx) * p[i].Y + (-cz * sx + sz * sy * cx) * p[i].Z;
                float qz = (-sy) * p[i].X + (cy * sx) * p[i].Y + (cy * cx) * p[i].Z;

                Vector3 q = new Vector3(qx, qy, qz);
                q = q + Translation + h0;
                PlatformPoints[i] = q;

                // 2. Inverse Kinematics
                Vector3 l = q - b[i];
                double L = l.LengthSquared() - (ROD_LENGTH * ROD_LENGTH) + (HORN_LENGTH * HORN_LENGTH);
                double M = 2 * HORN_LENGTH * (q.Z - b[i].Z);
                double N = 2 * HORN_LENGTH * (Math.Cos(BETA[i]) * (q.X - b[i].X) + Math.Sin(BETA[i]) * (q.Y - b[i].Y));

                double val = L / Math.Sqrt(M * M + N * N);
                if (val < -1) val = -1;
                if (val > 1) val = 1;

                Alpha[i] = Math.Asin(val) - Math.Atan2(N, M);

                // 3. Horn End Points for Drawing
                float ax = HORN_LENGTH * (float)Math.Cos(Alpha[i]) * (float)Math.Cos(BETA[i]) + b[i].X;
                float ay = HORN_LENGTH * (float)Math.Cos(Alpha[i]) * (float)Math.Sin(BETA[i]) + b[i].Y;
                float az = HORN_LENGTH * (float)Math.Sin(Alpha[i]) + b[i].Z;

                HornEndPoints[i] = new Vector3(ax, ay, az);
            }
        }

        private double ToRadians(double degrees) { return degrees * (Math.PI / 180.0); }
        public double GetAlphaDegree(int index) { return Alpha[index] * (180.0 / Math.PI); }
    }
}