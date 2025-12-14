import { GoogleGenerativeAI } from "@google/generative-ai";

const genAI = new GoogleGenerativeAI(process.env.GEMINI_API_KEY!);

export async function POST(req: Request) {
  const { content, targetLanguage } = await req.json();
  const langName = targetLanguage === 'ur' ? 'Urdu' : 'Hindi';

  const model = genAI.getGenerativeModel({ model: "gemini-2.0-flash" });

  const prompt = `Translate the following Markdown content to ${langName}. 
  CRITICAL RULE: DO NOT translate any code blocks (content between triple backticks). 
  Keep them exactly as is in English. 
  Preserve all other Markdown formatting.
  
  Content:
  ${content}`;

  const result = await model.generateContent(prompt);
  const response = await result.response;
  const text = response.text();

  return Response.json({ translatedContent: text });
}
