using System;
using System.Numerics; // Standard math library

namespace StewartPlatformPi
{
    public class StewartPlatform
    {
        // --- PHYSICAL CONSTANTS ---
        private readonly double[] BASE_ANGLES = { -50, -70, -170, -190, -290, -310 };
        private readonly double[] PLATFORM_ANGLES = { -54, -66, -174, -186, -294, -306 };

        // Beta angles in Radians
        private readonly double[] BETA = {
            Math.PI / 6, -5 * Math.PI / 6, -Math.PI / 2,
            Math.PI / 2, 5 * Math.PI / 6, -Math.PI / 6
        };

        // Dimensions (mm)
        public const float BASE_RADIUS = 76f;
        public const float PLATFORM_RADIUS = 60f;
        public const float HORN_LENGTH = 40f;
        public const float ROD_LENGTH = 130f;
        private const float INITIAL_HEIGHT = 120.28f;

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

            for (int i = 0; i < 6; i++)
            {
                float cx = (float)Math.Cos(Rotation.X); float sx = (float)Math.Sin(Rotation.X);
                float cy = (float)Math.Cos(Rotation.Y); float sy = (float)Math.Sin(Rotation.Y);
                float cz = (float)Math.Cos(Rotation.Z); float sz = (float)Math.Sin(Rotation.Z);

                float qx = (cz * cy) * p[i].X + (-sz * cx + cz * sy * sx) * p[i].Y + (sz * sx + cz * sy * cx) * p[i].Z;
                float qy = (sz * cy) * p[i].X + (cz * cx + sz * sy * sx) * p[i].Y + (-cz * sx + sz * sy * cx) * p[i].Z;
                float qz = (-sy) * p[i].X + (cy * sx) * p[i].Y + (cy * cx) * p[i].Z;

                Vector3 q = new Vector3(qx, qy, qz);
                q = q + Translation + h0;
                PlatformPoints[i] = q;

                Vector3 l = q - b[i];

                double L = l.LengthSquared() - (ROD_LENGTH * ROD_LENGTH) + (HORN_LENGTH * HORN_LENGTH);
                double M = 2 * HORN_LENGTH * (q.Z - b[i].Z);
                double N = 2 * HORN_LENGTH * (Math.Cos(BETA[i]) * (q.X - b[i].X) + Math.Sin(BETA[i]) * (q.Y - b[i].Y));

                double val = L / Math.Sqrt(M * M + N * N);
                if (val < -1) val = -1;
                if (val > 1) val = 1;

                Alpha[i] = Math.Asin(val) - Math.Atan2(N, M);

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