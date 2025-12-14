import { OpenAI } from "openai";
import { QdrantClient } from "@qdrant/js-client-rest";

const openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });
const qdrant = new QdrantClient({ url: process.env.QDRANT_URL });

export async function POST(req: Request) {
  const { messages } = await req.json();
  const lastMessage = messages[messages.length - 1].content;

  // 1. Embed query
  const embedding = await openai.embeddings.create({
    model: "text-embedding-3-small",
    input: lastMessage,
  });

  // 2. Search Qdrant
  const searchResult = await qdrant.search("textbook_knowledge", {
    vector: embedding.data[0].embedding,
    limit: 3,
  });

  // 3. Construct Context
  const context = searchResult.map((item) => item.payload?.content).join("\n\n");

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
