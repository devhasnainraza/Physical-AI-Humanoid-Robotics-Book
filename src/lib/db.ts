import { neon, neonConfig } from '@neondatabase/serverless';

// FORCE GLOBAL FETCH
neonConfig.useFetchPolyfill = true;

// DISABLE ALL LOCALHOST LOGIC
neonConfig.pipelineConnect = false;
neonConfig.coalesceGC = true;

const getDb = (connectionString: string) => {
  if (!connectionString) {
    console.error("Neon: No connection string provided.");
    return null;
  }

  // FORCE CLOUD ENDPOINT
  try {
      // Extract hostname: postgres://user:pass@ep-xyz.aws.neon.tech/db
      const match = connectionString.match(/@([\w.-]+)\//);
      if (match && match[1]) {
          const host = match[1];
          neonConfig.fetchEndpoint = `https://${host}/sql`;
          console.log(`Neon: Configured to hit ${neonConfig.fetchEndpoint}`);
      }
  } catch (e) {
      console.error("Neon: Failed to parse host", e);
  }

  return neon(connectionString);
};

export default getDb;