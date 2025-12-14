import React, { useState, useEffect, useRef } from 'react';
import { GoogleGenerativeAI } from "@google/generative-ai";

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<{role: 'user'|'model', content: string}[]>([
    { role: 'model', content: 'Hi! I am Cortex. Ask me anything about this chapter.' }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [context, setContext] = useState('');
  
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Load Page Content as Context on Mount/Navigation
  useEffect(() => {
    const updateContext = () => {
        const contentDiv = document.querySelector('.theme-doc-markdown');
        if (contentDiv) {
            // Cleanup text to save tokens
            setContext(contentDiv.textContent || "");
        }
    };
    
    updateContext();
    // Listen for navigation changes (hacky but works for SPA)
    const observer = new MutationObserver(updateContext);
    const main = document.querySelector('main');
    if (main) observer.observe(main, { childList: true, subtree: true });
    
    return () => observer.disconnect();
  }, []);

  // Listen for Text Selection
  useEffect(() => {
    const handleSelection = (e: CustomEvent) => {
        const text = e.detail;
        if (text) {
            setIsOpen(true);
            setInput(`Explain this: "${text}"`);
        }
    };
    // @ts-ignore
    window.addEventListener('textbook-selection', handleSelection);
    return () => {
        // @ts-ignore
        window.removeEventListener('textbook-selection', handleSelection);
    };
  }, []);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!input.trim()) return;

    const userMsg = input;
    setMessages(prev => [...prev, { role: 'user', content: userMsg }]);
    setInput('');
    setLoading(true);

    // Get Key
    let apiKey = localStorage.getItem('GEMINI_API_KEY');
    if (!apiKey) {
        apiKey = prompt("To chat, please enter your Gemini API Key (saved locally):");
        if (apiKey) localStorage.setItem('GEMINI_API_KEY', apiKey);
        else {
            setMessages(prev => [...prev, { role: 'model', content: 'I need an API Key to function. Please try again.' }]);
            setLoading(false);
            return;
        }
    }

    try {
        const genAI = new GoogleGenerativeAI(apiKey);
        const model = genAI.getGenerativeModel({ model: "gemini-2.5-flash" });

        const prompt = `
        You are a helpful AI Tutor for a Robotics Textbook.
        
        CONTEXT FROM CURRENT CHAPTER:
        """${context.slice(0, 20000)}""" 
        (Truncated if too long)

        USER QUESTION: "${userMsg}"

        Answer the user based on the context above. Be concise and educational.
        `;

        const result = await model.generateContent(prompt);
        const response = await result.response;
        const text = response.text();

        setMessages(prev => [...prev, { role: 'model', content: text }]);
    } catch (error: any) {
        console.error(error);
        setMessages(prev => [...prev, { role: 'model', content: `Error: ${error.message}` }]);
    }

    setLoading(false);
  };

  if (!isOpen) {
    return (
      <button 
        onClick={() => setIsOpen(true)}
        className="fixed bottom-6 right-6 w-14 h-14 bg-gradient-to-r from-blue-600 to-teal-500 hover:scale-110 transition-transform rounded-full shadow-2xl flex items-center justify-center text-white z-50 group"
      >
        <svg className="w-8 h-8" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
        </svg>
        {/* Notification Dot */}
        <span className="absolute top-0 right-0 w-4 h-4 bg-red-500 rounded-full animate-ping"></span>
        <span className="absolute top-0 right-0 w-4 h-4 bg-red-500 rounded-full border-2 border-white"></span>
      </button>
    );
  }

  return (
    <div className="fixed bottom-6 right-6 w-[350px] md:w-[400px] h-[500px] bg-white dark:bg-gray-900 border border-gray-200 dark:border-gray-700 rounded-2xl shadow-2xl flex flex-col z-50 overflow-hidden animate-fade-in-up">
      {/* Header */}
      <div className="p-4 bg-gradient-to-r from-blue-600 to-teal-600 text-white flex justify-between items-center">
        <div className="flex items-center gap-2">
            <span className="text-xl">🤖</span>
            <div>
                <h3 className="font-bold text-sm">Cortex AI Tutor</h3>
                <p className="text-xs opacity-80">Ask about this chapter</p>
            </div>
        </div>
        <button onClick={() => setIsOpen(false)} className="hover:bg-white/20 p-1 rounded">
            <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" /></svg>
        </button>
      </div>

      {/* Messages */}
      <div className="flex-1 overflow-y-auto p-4 space-y-4 bg-gray-50 dark:bg-black/20">
        {messages.map((m, i) => (
            <div key={i} className={`flex ${m.role === 'user' ? 'justify-end' : 'justify-start'}`}>
                <div className={`max-w-[80%] p-3 rounded-2xl text-sm ${
                    m.role === 'user' 
                    ? 'bg-blue-600 text-white rounded-br-none' 
                    : 'bg-white dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-bl-none shadow-sm'
                }`}>
                    {m.content}
                </div>
            </div>
        ))}
        {loading && (
            <div className="flex justify-start">
                <div className="bg-white dark:bg-gray-800 p-3 rounded-2xl rounded-bl-none shadow-sm flex gap-1">
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce"></div>
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce delay-100"></div>
                    <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce delay-200"></div>
                </div>
            </div>
        )}
        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <form onSubmit={handleSubmit} className="p-3 bg-white dark:bg-gray-900 border-t border-gray-200 dark:border-gray-700">
        <div className="relative">
            <input
                value={input}
                onChange={(e) => setInput(e.target.value)}
                placeholder="Ask a question..."
                className="w-full pl-4 pr-10 py-3 bg-gray-100 dark:bg-gray-800 rounded-xl outline-none focus:ring-2 focus:ring-blue-500 text-sm"
            />
            <button 
                type="submit" 
                disabled={loading || !input.trim()}
                className="absolute right-2 top-1/2 -translate-y-1/2 p-1.5 bg-blue-600 text-white rounded-lg hover:bg-blue-700 disabled:opacity-50 disabled:hover:bg-blue-600 transition-colors"
            >
                <svg className="w-4 h-4" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M5 12h14M12 5l7 7-7 7" /></svg>
            </button>
        </div>
      </form>
    </div>
  );
}
