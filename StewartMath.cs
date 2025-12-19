using System;
using System.Numerics;

namespace StewartPlatformPi
{
    public class StewartPlatform
    {
        // --- PHYSICAL CONSTANTS ---
        private readonly double[] BASE_ANGLES = { 0, 60, 120, 180, 240, 300 };
        private readonly double[] PLATFORM_ANGLES = { 345, 75, 105, 195, 225, 315 };

        // Even servos (+90), Odd servos (-90) relative to radius
        private readonly double[] BETA = {
            Math.PI / 2, -Math.PI / 6, 7 * Math.PI / 6,
            Math.PI / 2, 11 * Math.PI / 6, 7 * Math.PI / 6
        };

        // Dimensions (mm)
        public const float BASE_RADIUS = 300f;
        public const float PLATFORM_RADIUS = 270f;
        public const float HORN_LENGTH = 38.0f;
        public const float ROD_LENGTH = 213.485f;
        private const float INITIAL_HEIGHT = 180.0f;

        // Public Data
        public Vector3[] BasePoints { get; private set; } = new Vector3[6];
        public Vector3[] PlatformPoints { get; private set; } = new Vector3[6];
        public Vector3[] HornEndPoints { get; private set; } = new Vector3[6];
        public double[] Alpha { get; private set; } = new double[6];

        // NEW: Flag to tell UI if we hit a limit
        public bool IsOutOfRange { get; private set; } = false;

        // Internal
        private Vector3[] b = new Vector3[6];
        private Vector3[] p = new Vector3[6];
        private Vector3 Translation;
        private Vector3 Rotation;

        public StewartPlatform()
        {
            for (int i = 0; i < 6; i++)
            {
                float xb = BASE_RADIUS * (float)Math.Cos(ToRadians(BASE_ANGLES[i]));
                float yb = BASE_RADIUS * (float)Math.Sin(ToRadians(BASE_ANGLES[i]));
                b[i] = new Vector3(xb, yb, 0);
                BasePoints[i] = b[i];

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
            IsOutOfRange = false; // Reset flag

            for (int i = 0; i < 6; i++)
            {
                // 1. Calculate q[i] (Platform Joint World Position)
                float cx = (float)Math.Cos(Rotation.X); float sx = (float)Math.Sin(Rotation.X);
                float cy = (float)Math.Cos(Rotation.Y); float sy = (float)Math.Sin(Rotation.Y);
                float cz = (float)Math.Cos(Rotation.Z); float sz = (float)Math.Sin(Rotation.Z);

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

                // CHECK FOR LIMITS
                if (val < -1) { val = -1; IsOutOfRange = true; }
                if (val > 1) { val = 1; IsOutOfRange = true; }

                Alpha[i] = Math.Asin(val) - Math.Atan2(N, M);

                // Check Degree Limits (0 to 180)
                double deg = Alpha[i] * (180.0 / Math.PI);
                if (deg < 0 || deg > 180) IsOutOfRange = true;

                // 3. Horn End Points
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