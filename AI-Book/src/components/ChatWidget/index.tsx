import React, { useEffect, useState } from 'react';
import { useChat } from '@ai-sdk/react';

export default function ChatWidget() {
  const { messages, input, handleInputChange, handleSubmit, setInput } = useChat({
    api: '/api/chat', // Assuming this route is proxied or handled
  });
  const [isOpen, setIsOpen] = useState(false);

  // Listen for custom event from Layout
  useEffect(() => {
    const handleSelection = (e: CustomEvent) => {
      const selectedText = e.detail;
      if (selectedText) {
        setIsOpen(true);
        setInput(`Explain ${selectedText} specifically for a humanoid robot.`);
      }
    };
    
    // @ts-ignore
    window.addEventListener('textbook-selection', handleSelection);
    return () => {
      // @ts-ignore
      window.removeEventListener('textbook-selection', handleSelection);
    };
  }, [setInput]);

  if (!isOpen) {
    return (
      <button 
        className="fixed bottom-4 right-4 bg-green-600 text-white p-3 rounded-full shadow-lg"
        onClick={() => setIsOpen(true)}
      >
        Chat
      </button>
    );
  }

  return (
    <div className="fixed bottom-4 right-4 w-80 h-96 bg-white border border-gray-300 rounded shadow-xl flex flex-col">
      <div className="flex justify-between p-2 bg-gray-100 border-b">
        <span>AI Assistant</span>
        <button onClick={() => setIsOpen(false)}>X</button>
      </div>
      <div className="flex-1 overflow-auto p-2">
        {messages.map(m => (
          <div key={m.id} className={`mb-2 ${m.role === 'user' ? 'text-right' : 'text-left'}`}>
            <span className={`inline-block p-2 rounded ${m.role === 'user' ? 'bg-blue-100' : 'bg-gray-100'}`}>
              {m.content}
            </span>
          </div>
        ))}
      </div>
      <form onSubmit={handleSubmit} className="p-2 border-t">
        <input
          value={input}
          onChange={handleInputChange}
          placeholder="Ask something..."
          className="w-full border p-1 rounded"
        />
      </form>
    </div>
  );
}
