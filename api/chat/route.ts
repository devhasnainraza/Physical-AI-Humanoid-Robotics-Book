import { OpenAI } from "openai";
import { QdrantClient } from "@qdrant/js-client-rest";
import { CohereClient } from "cohere-ai";

const openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });
const cohere = new CohereClient({ token: process.env.COHERE_API_KEY });
const qdrant = new QdrantClient({ url: process.env.QDRANT_URL, apiKey: process.env.QDRANT_API_KEY });

export async function POST(req: Request) {
  const { messages } = await req.json();
  const lastMessage = messages[messages.length - 1].content;

  // 1. Embed query using Cohere
  const embedResult = await cohere.embed({
    model: "embed-english-v3.0",
    texts: [lastMessage],
    inputType: "search_query",
  });

  const embedding = embedResult.embeddings[0]; // array of numbers

  // 2. Search Qdrant
  const searchResult = await qdrant.search("textbook_knowledge", {
    vector: embedding,
    limit: 3,
  });

  // 3. Construct Context
  const context = searchResult.map((item) => item.payload?.content || item.payload?.text).join("\n\n");

  // 4. Generate Answer
  const completion = await openai.chat.completions.create({
    model: "gpt-4o",
    messages: [
      { role: "system", content: `You are a helper for a Robotics textbook. Use this context: ${context}` },
      ...messages,
    ],
  });

  return Response.json(completion.choices[0].message);
}
