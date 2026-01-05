// @ts-nocheck
/**
 * Better Auth standalone server (Express)
 * Features:
 * - Username/password auth (minPasswordLength 8)
 * - Google OAuth
 * - Email OTP + optional 2FA OTP
 *
 * Env:
 * AUTH_DATABASE_URL=postgres://user:pass@host:port/db
 * AUTH_GOOGLE_CLIENT_ID=...
 * AUTH_GOOGLE_CLIENT_SECRET=...
 * AUTH_JWT_SECRET=some-long-random-secret
 * PORT=3000
 *
 * Notes:
 * - Replace sendEmail with your SMTP/Resend provider for production.
 * - Frontend authClient is configured to call same-origin /api/auth.
 */

import "dotenv/config";
import express from "express";
import cors from "cors";
import { betterAuth } from "better-auth";
import { emailOTP, twoFactor } from "better-auth/plugins";
import { Pool } from "pg";

// Replace with your email provider (Resend/SMTP/etc.)
async function sendEmail({ to, subject, text }: { to: string; subject: string; text: string }) {
  console.log("[email send]", { to, subject, text });
}

const pool = new Pool({
  connectionString: process.env.AUTH_DATABASE_URL,
});

const auth = betterAuth({
  database: pool,
  jwt: {
    secret: process.env.AUTH_JWT_SECRET || "dev-secret-change-me",
  },
  providers: {
    username: { minPasswordLength: 8 },
  },
  socialProviders: {
    google: {
      clientId: process.env.AUTH_GOOGLE_CLIENT_ID || "",
      clientSecret: process.env.AUTH_GOOGLE_CLIENT_SECRET || "",
    },
  },
  plugins: [
    emailOTP({
      async sendVerificationOTP({ email, otp, type }) {
        await sendEmail({
          to: email,
          subject: `Your ${type} code`,
          text: `Your code is: ${otp}`,
        });
      },
    }),
    twoFactor({
      otpOptions: {
        async sendOTP({ user, otp }) {
          await sendEmail({
            to: user.email,
            subject: "Your 2FA code",
            text: `2FA code: ${otp}`,
          });
        },
      },
    }),
  ],
  user: {
    additionalFields: {
      hardware_profile: {
        type: "string",
        defaultValue: "Cloud_Simulation_Only",
      },
    },
  },
});

const app = express();
app.use(cors({ origin: "*", credentials: true }));
app.use(express.json());

// Better Auth handler
app.use("/api/auth", auth.handler);

const port = process.env.PORT || 3000;
app.listen(port, () => {
  console.log(`Better Auth server listening on http://localhost:${port}/api/auth`);
});

