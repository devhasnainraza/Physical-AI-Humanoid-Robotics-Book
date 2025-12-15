/**
 * Setup script for Qdrant Cloud collection
 * Run with: node scripts/setup_qdrant.js
 */
const { QdrantClient } = require('@qdrant/js-client-rest');

const QDRANT_URL = process.env.QDRANT_URL || 'https://your-cluster.qdrant.io';
const QDRANT_API_KEY = process.env.QDRANT_API_KEY;
const COLLECTION_NAME = 'textbook_knowledge';

if (!QDRANT_API_KEY) {
  console.error('Error: QDRANT_API_KEY environment variable is required');
  console.log('Set it with: export QDRANT_API_KEY=your-key');
  process.exit(1);
}

const client = new QdrantClient({ 
  url: QDRANT_URL,
  apiKey: QDRANT_API_KEY
});

async function setup() {
  try {
    // Check if collection exists
    try {
      const collection = await client.getCollection(COLLECTION_NAME);
      console.log(`✅ Collection '${COLLECTION_NAME}' already exists`);
      console.log(`   Points: ${collection.points_count}`);
      console.log(`   Vector size: ${collection.config.params.vectors.size}`);
      return;
    } catch (error) {
      if (error.status === 404) {
        console.log(`Collection '${COLLECTION_NAME}' not found, creating...`);
      } else {
        throw error;
      }
    }

    // Create collection
    const result = await client.createCollection(COLLECTION_NAME, {
      vectors: {
        size: 1536, // OpenAI text-embedding-3-small dimension
        distance: 'Cosine',
      },
    });
    
    console.log(`✅ Collection '${COLLECTION_NAME}' created successfully`);
    console.log(`   URL: ${QDRANT_URL}`);
    console.log(`   Vector size: 1536 (OpenAI text-embedding-3-small)`);
  } catch (error) {
    console.error('❌ Error creating collection:', error.message);
    if (error.status === 401) {
      console.error('   Authentication failed. Check your QDRANT_API_KEY.');
    } else if (error.status === 404) {
      console.error('   Cluster not found. Check your QDRANT_URL.');
    }
    process.exit(1);
  }
}

setup();
