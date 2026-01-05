require("dotenv/config");
const express = require("express");
const cors = require("cors");
const { betterAuth } = require("better-auth");
const { emailOTP, twoFactor } = require("better-auth/plugins");
const { Pool } = require("pg");

// Polyfill for Fetch API Request/Response in Node.js
let Request, Response, Headers;
try {
  // Node 18+ has fetch but might need Request/Response from undici
  if (typeof globalThis.Request === "undefined" || typeof globalThis.Headers === "undefined") {
    const undici = require("undici");
    Request = undici.Request;
    Response = undici.Response;
    Headers = undici.Headers;
    globalThis.Request = Request;
    globalThis.Response = Response;
    globalThis.Headers = Headers;
  } else {
    Request = globalThis.Request;
    Response = globalThis.Response;
    Headers = globalThis.Headers;
  }
} catch (e) {
  console.error("Failed to load undici, install it: npm install undici");
  process.exit(1);
}

// Real email sending with Resend
async function sendEmail({ to, subject, text, html }) {
  const RESEND_API_KEY = process.env.RESEND_API_KEY;
  
  if (!RESEND_API_KEY) {
    console.warn("[email] RESEND_API_KEY not set, email not sent. To:", to, "Subject:", subject);
    console.log("[email preview]", { to, subject, text });
    return;
  }

  try {
    const { Resend } = require("resend");
    const resend = new Resend(RESEND_API_KEY);
    
    const fromEmail = process.env.RESEND_FROM_EMAIL || "onboarding@resend.dev";
    
    const result = await resend.emails.send({
      from: fromEmail,
      to: to,
      subject: subject,
      text: text,
      html: html || text.replace(/\n/g, "<br>"),
    });

    if (result.error) {
      console.error("[email error]", result.error);
    } else {
      console.log("[email sent]", { to, subject, id: result.data?.id });
    }
  } catch (error) {
    console.error("[email send failed]", error);
  }
}

const pool = new Pool({
  connectionString: process.env.AUTH_DATABASE_URL,
});

// Normalize AUTH_BASE_URL: allow either "http://host:port" or "http://host:port/api/auth"
const rawBaseUrl = process.env.AUTH_BASE_URL || "http://localhost:3000";
const normalizedBaseUrl = rawBaseUrl.replace(/\/api\/auth\/?$/i, "");

const auth = betterAuth({
  baseURL: normalizedBaseUrl,
  basePath: "/api/auth",
  database: pool,
  jwt: {
    secret: process.env.AUTH_JWT_SECRET || "dev-secret-change-me",
  },
  cookies: {
    // For local dev, allow non-secure cookies. In production set secure=true with HTTPS.
    secure: process.env.NODE_ENV === "production",
    sameSite: "lax",
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
        const subject = type === "sign-in" 
          ? "Your login code for Cortex-H1" 
          : "Verify your email for Cortex-H1";
        const html = `
          <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px;">
            <h2 style="color: #7c3aed;">Cortex-H1 Authentication</h2>
            <p>Your verification code is:</p>
            <div style="background: #f3f4f6; padding: 20px; text-align: center; font-size: 32px; font-weight: bold; letter-spacing: 8px; margin: 20px 0; border-radius: 8px;">
              ${otp}
            </div>
            <p style="color: #6b7280; font-size: 14px;">This code will expire in 10 minutes.</p>
            <p style="color: #6b7280; font-size: 14px;">If you didn't request this code, please ignore this email.</p>
          </div>
        `;
        await sendEmail({
          to: email,
          subject: subject,
          text: `Your verification code is: ${otp}. This code will expire in 10 minutes.`,
          html: html,
        });
      },
    }),
    twoFactor({
      otpOptions: {
        async sendOTP({ user, otp }) {
          await sendEmail({
            to: user.email,
            subject: "Your 2FA code for Cortex-H1",
            text: `Your 2FA code is: ${otp}`,
            html: `
              <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px;">
                <h2 style="color: #7c3aed;">Two-Factor Authentication</h2>
                <p>Hello ${user.name || user.email},</p>
                <p>Your 2FA code is:</p>
                <div style="background: #f3f4f6; padding: 20px; text-align: center; font-size: 32px; font-weight: bold; letter-spacing: 8px; margin: 20px 0; border-radius: 8px;">
                  ${otp}
                </div>
                <p style="color: #6b7280; font-size: 14px;">This code will expire in 5 minutes.</p>
              </div>
            `,
          });
        },
      },
    }),
  ],
  emailVerification: {
    sendVerificationEmail: async ({ user, url }) => {
      await sendEmail({
        to: user.email,
        subject: "Verify your email for Cortex-H1",
        text: `Click this link to verify your email: ${url}`,
        html: `
          <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px;">
            <h2 style="color: #7c3aed;">Verify your email</h2>
            <p>Hello ${user.name || user.email},</p>
            <p>Click the button below to verify your email address:</p>
            <div style="text-align: center; margin: 30px 0;">
              <a href="${url}" style="background: #7c3aed; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; display: inline-block; font-weight: bold;">Verify Email</a>
            </div>
            <p style="color: #6b7280; font-size: 14px;">Or copy and paste this link into your browser:</p>
            <p style="color: #6b7280; font-size: 12px; word-break: break-all;">${url}</p>
          </div>
        `,
      });
    },
  },
  passwordReset: {
    sendResetPasswordEmail: async ({ user, url }) => {
      await sendEmail({
        to: user.email,
        subject: "Reset your password for Cortex-H1",
        text: `Click this link to reset your password: ${url}`,
        html: `
          <div style="font-family: Arial, sans-serif; max-width: 600px; margin: 0 auto; padding: 20px;">
            <h2 style="color: #7c3aed;">Reset your password</h2>
            <p>Hello ${user.name || user.email},</p>
            <p>Click the button below to reset your password:</p>
            <div style="text-align: center; margin: 30px 0;">
              <a href="${url}" style="background: #7c3aed; color: white; padding: 12px 24px; text-decoration: none; border-radius: 6px; display: inline-block; font-weight: bold;">Reset Password</a>
            </div>
            <p style="color: #6b7280; font-size: 14px;">Or copy and paste this link into your browser:</p>
            <p style="color: #6b7280; font-size: 12px; word-break: break-all;">${url}</p>
            <p style="color: #dc2626; font-size: 14px; margin-top: 20px;">⚠️ If you didn't request a password reset, please ignore this email.</p>
          </div>
        `,
      });
    },
  },
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
// Important: allow credentials and set the exact frontend origin
const FRONTEND_ORIGIN =
  process.env.FRONTEND_ORIGIN ||
  process.env.AUTH_FRONTEND_ORIGIN ||
  "http://localhost:3001";

app.set("trust proxy", 1);
app.use(
  cors({
    origin: FRONTEND_ORIGIN === "*" ? true : FRONTEND_ORIGIN,
    credentials: true,
    allowedHeaders: ["Content-Type", "Authorization"],
  })
);
app.use(express.json());

// Convert Express req/res to Fetch API Request/Response for Better Auth
app.use("/api/auth", async (req, res) => {
  // Build full URL
  const protocol = req.protocol || "http";
  const host = req.get("host") || "localhost:3000";
  const url = `${protocol}://${host}${req.originalUrl || req.url}`;

  // Create Fetch API compatible Request
  const headers = new Headers();
  Object.keys(req.headers).forEach((key) => {
    const value = req.headers[key];
    if (typeof value === "string") {
      headers.set(key, value);
    } else if (Array.isArray(value)) {
      value.forEach((v) => headers.append(key, v));
    }
  });

  const request = new Request(url, {
    method: req.method,
    headers: headers,
    body: req.method !== "GET" && req.method !== "HEAD" ? JSON.stringify(req.body) : undefined,
  });

  try {
    // Call Better Auth handler with Fetch API Request
    const response = await auth.handler(request);

    // Convert Fetch Response back to Express response
    res.status(response.status);
    
    // Copy headers
    response.headers.forEach((value, key) => {
      res.setHeader(key, value);
    });

    // Send body
    const body = await response.text();
    res.send(body);
  } catch (error) {
    console.error("Auth handler error:", error);
    res.status(500).json({ error: "Internal server error", message: error.message });
  }
});

const port = process.env.PORT || 3000;
app.listen(port, () => {
  console.log(`Better Auth server listening on http://localhost:${port}/api/auth`);
});


