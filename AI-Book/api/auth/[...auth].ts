import { betterAuth } from "better-auth";
import { Pool } from "pg";

const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
});

export const auth = betterAuth({
  database: pool,
  providers: {
    username: {
       minPasswordLength: 8,
    }, 
    // Add other providers here
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
