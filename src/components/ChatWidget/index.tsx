import React, { useState, useEffect, useRef } from 'react';

interface Message {
  role: 'user' | 'model';
  content: string;
  selectedText?: string;
  sources?: string[];
}

export default function ChatWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    { 
      role: 'model', 
      content: 'Hi! I am Cortex, your AI tutor for Physical AI & Humanoid Robotics. I use RAG to answer questions from the textbook. Try selecting text from the book to ask me about it!' 
    }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [backendUrl, setBackendUrl] = useState('http://localhost:8000');
  const [showSettings, setShowSettings] = useState(false);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState<string>('');
  const [showSelectedText, setShowSelectedText] = useState(false);
  
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Load backend URL preference
  useEffect(() => {
    const savedUrl = localStorage.getItem('rag_backend_url');
    if (savedUrl) setBackendUrl(savedUrl);
  }, []);

  // Auto-scroll
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Listen for Text Selection
  useEffect(() => {
    const handleSelection = (e: CustomEvent) => {
      const text = e.detail;
      if (text && text.length > 0) {
        setSelectedText(text);
        setShowSelectedText(true);
        setIsOpen(true);
        // Pre-fill input with a helpful prompt
        setInput(`Explain this: "${text.substring(0, 100)}${text.length > 100 ? '...' : ''}"`);
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
    const currentSelectedText = selectedText;
    
    // Add user message to UI
    setMessages(prev => [...prev, { 
      role: 'user', 
      content: userMsg,
      selectedText: currentSelectedText || undefined
    }]);
    setInput('');
    setSelectedText(''); // Clear selected text after sending
    setShowSelectedText(false);
    setLoading(true);

    try {
      // Extract selected text from message if it's in the format "Explain this: "..."
      let extractedSelectedText = currentSelectedText;
      if (!extractedSelectedText) {
        const match = userMsg.match(/Explain this: "(.*?)"/);
        if (match) extractedSelectedText = match[1];
      }

      const requestBody: any = {
        message: userMsg,
        selected_text: extractedSelectedText || "",
      };

      // Include conversation ID if we have one
      if (conversationId) {
        requestBody.conversation_id = conversationId;
      }

      const res = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(requestBody)
      });

      if (!res.ok) {
        const errorData = await res.json().catch(() => ({ detail: 'Unknown error' }));
        throw new Error(errorData.detail || `HTTP ${res.status}: Backend connection failed`);
      }
      
      const data = await res.json();
      
      // Store conversation ID for future messages
      if (data.conversation_id && !conversationId) {
        setConversationId(data.conversation_id);
      }
      
      // Add assistant response
      setMessages(prev => [...prev, { 
        role: 'model', 
        content: data.response,
        sources: data.sources || undefined
      }]);
    } catch (error: any) {
      console.error('Chat error:', error);
      setMessages(prev => [...prev, { 
        role: 'model', 
        content: `Error: ${error.message}. ${!error.message.includes('HTTP') ? `Is the backend running at ${backendUrl}?` : ''}` 
      }]);
    }

    setLoading(false);
  };

  const handleNewConversation = () => {
    setMessages([{ 
      role: 'model', 
      content: 'Hi! I am Cortex, your AI tutor for Physical AI & Humanoid Robotics. I use RAG to answer questions from the textbook. Try selecting text from the book to ask me about it!' 
    }]);
    setConversationId(null);
    setSelectedText('');
    setShowSelectedText(false);
  };

  if (!isOpen) {
    return (
      <button 
        onClick={() => setIsOpen(true)}
        className="fixed bottom-6 right-6 w-14 h-14 bg-gradient-to-r from-purple-600 to-indigo-500 hover:scale-110 transition-transform rounded-full shadow-2xl flex items-center justify-center text-white z-50 group"
        aria-label="Open chat"
      >
        <svg className="w-7 h-7" fill="none" viewBox="0 0 24 24" stroke="currentColor">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
        </svg>
      </button>
    );
  }

  return (
    <div className="fixed bottom-6 right-6 w-[350px] md:w-[400px] h-[500px] bg-white dark:bg-gray-900 border border-gray-200 dark:border-gray-700 rounded-2xl shadow-2xl flex flex-col z-50 overflow-hidden animate-fade-in-up">
      {/* Header */}
      <div className="p-4 bg-gradient-to-r from-purple-600 to-indigo-600 text-white flex justify-between items-center">
        <div className="flex items-center gap-2">
          <span className="text-xl">🧠</span>
          <div>
            <h3 className="font-bold text-sm">Cortex RAG Assistant</h3>
            <p className="text-xs opacity-80">OpenAI + Qdrant + Neon</p>
          </div>
        </div>
        <div className="flex gap-2">
          <button 
            onClick={handleNewConversation} 
            className="hover:bg-white/20 p-1 rounded text-xs" 
            title="New conversation"
          >
            🆕
          </button>
          <button 
            onClick={() => setShowSettings(!showSettings)} 
            className="hover:bg-white/20 p-1 rounded text-xs"
            title="Settings"
          >
            ⚙️
          </button>
          <button 
            onClick={() => setIsOpen(false)} 
            className="hover:bg-white/20 p-1 rounded"
            title="Close"
          >
            ✖
          </button>
        </div>
      </div>

      {/* Selected Text Indicator */}
      {showSelectedText && selectedText && (
        <div className="px-4 py-2 bg-purple-50 dark:bg-purple-900/20 border-b border-purple-200 dark:border-purple-800 text-xs">
          <div className="flex items-start justify-between gap-2">
            <div className="flex-1">
              <span className="font-semibold text-purple-700 dark:text-purple-300">Selected text:</span>
              <p className="text-purple-600 dark:text-purple-400 mt-1 line-clamp-2">
                "{selectedText.substring(0, 150)}{selectedText.length > 150 ? '...' : ''}"
              </p>
            </div>
            <button 
              onClick={() => {
                setSelectedText('');
                setShowSelectedText(false);
              }}
              className="text-purple-500 hover:text-purple-700 dark:text-purple-400 dark:hover:text-purple-200"
            >
              ✕
            </button>
          </div>
        </div>
      )}

      {/* Settings */}
      {showSettings && (
        <div className="p-3 bg-gray-100 dark:bg-gray-800 text-xs border-b border-gray-200 dark:border-gray-700">
          <label className="block mb-1 font-bold">Backend URL:</label>
          <input 
            value={backendUrl}
            onChange={(e) => {
              setBackendUrl(e.target.value);
              localStorage.setItem('rag_backend_url', e.target.value);
            }}
            className="w-full p-2 rounded border border-gray-300 dark:border-gray-600 dark:bg-gray-700 text-xs"
            placeholder="http://localhost:8000"
          />
          {conversationId && (
            <div className="mt-2 text-xs text-gray-600 dark:text-gray-400">
              Conversation ID: <code className="text-xs">{conversationId.substring(0, 8)}...</code>
            </div>
          )}
        </div>
      )}

      {/* Messages */}
      <div className="flex-1 overflow-y-auto p-4 space-y-4 bg-gray-50 dark:bg-black/20">
        {messages.map((m, i) => (
          <div key={i} className={`flex ${m.role === 'user' ? 'justify-end' : 'justify-start'}`}>
            <div className={`max-w-[85%] ${m.role === 'user' ? 'flex flex-col items-end' : 'flex flex-col items-start'}`}>
              {m.selectedText && m.role === 'user' && (
                <div className="mb-1 px-2 py-1 bg-purple-100 dark:bg-purple-900/30 rounded text-xs text-purple-700 dark:text-purple-300 max-w-full">
                  <span className="font-semibold">Context:</span> "{m.selectedText.substring(0, 80)}{m.selectedText.length > 80 ? '...' : ''}"
                </div>
              )}
              <div className={`p-3 rounded-2xl text-sm ${
                m.role === 'user' 
                  ? 'bg-purple-600 text-white rounded-br-none' 
                  : 'bg-white dark:bg-gray-800 border border-gray-200 dark:border-gray-700 rounded-bl-none shadow-sm'
              }`}>
                <div className="whitespace-pre-wrap break-words">{m.content}</div>
                {m.sources && m.sources.length > 0 && (
                  <div className="mt-2 pt-2 border-t border-gray-200 dark:border-gray-600">
                    <div className="text-xs font-semibold text-gray-600 dark:text-gray-400 mb-1">Sources:</div>
                    <div className="flex flex-wrap gap-1">
                      {m.sources.map((source, idx) => (
                        <span 
                          key={idx} 
                          className="text-xs px-2 py-1 bg-gray-100 dark:bg-gray-700 rounded text-gray-700 dark:text-gray-300"
                        >
                          {source}
                        </span>
                      ))}
                    </div>
                  </div>
                )}
              </div>
            </div>
          </div>
        ))}
        {loading && (
          <div className="flex justify-start">
            <div className="bg-white dark:bg-gray-800 p-3 rounded-2xl rounded-bl-none shadow-sm flex gap-1">
              <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce"></div>
              <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0.1s' }}></div>
              <div className="w-2 h-2 bg-gray-400 rounded-full animate-bounce" style={{ animationDelay: '0.2s' }}></div>
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
            placeholder={selectedText ? "Ask about the selected text..." : "Ask about the book..."}
            className="w-full pl-4 pr-10 py-3 bg-gray-100 dark:bg-gray-800 rounded-xl outline-none focus:ring-2 focus:ring-purple-500 text-sm"
            disabled={loading}
          />
          <button 
            type="submit" 
            disabled={loading || !input.trim()}
            className="absolute right-2 top-1/2 -translate-y-1/2 p-1.5 bg-purple-600 text-white rounded-lg hover:bg-purple-700 disabled:opacity-50 disabled:hover:bg-purple-600 transition-colors"
            aria-label="Send message"
          >
            ➞
          </button>
        </div>
        {selectedText && !showSelectedText && (
          <button
            type="button"
            onClick={() => setShowSelectedText(true)}
            className="mt-2 text-xs text-purple-600 dark:text-purple-400 hover:underline"
          >
            📎 Using selected text as context
          </button>
        )}
      </form>
    </div>
  );
}
