const { QdrantClient } = require('@qdrant/js-client-rest');

const client = new QdrantClient({ url: 'http://localhost:6333' });

async function setup() {
  try {
    const result = await client.createCollection('textbook_knowledge', {
      vectors: {
        size: 1536, // OpenAI text-embedding-3-small
        distance: 'Cosine',
      },
    });
    console.log('Collection created:', result);
  } catch (error) {
    console.error('Error creating collection:', error);
  }
}

setup();
