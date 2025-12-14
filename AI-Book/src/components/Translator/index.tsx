import React, { useState } from 'react';

export default function Translator() {
  const [loading, setLoading] = useState(false);

  const translatePage = async (lang: 'ur' | 'hi') => {
    setLoading(true);
    // Find the main content
    const contentDiv = document.querySelector('.theme-doc-markdown');
    if (!contentDiv) {
      alert('Content not found');
      setLoading(false);
      return;
    }

    // For a robust implementation, we'd fetch the source MDX. 
    // Here we'll take innerText but that loses formatting.
    // Better strategy: Clone the node, preserve structure, translate text nodes.
    // BUT spec says "Server-Side Agent... translate Markdown content".
    // Since we don't have the MD content easily, let's assume we send the text content 
    // and replace it, OR ideally we would hook into Docusaurus data.
    // Given constraints, I will implement a visual replacement of text nodes.
    
    // Simplification for prototype:
    // We will send the innerHTML (risky but preserves structure) and ask Gemini to return translated HTML?
    // The Prompt said "translate Markdown content". 
    // Let's try to get the textContent and replace it? No, that kills structure.
    
    // Fallback Plan: Send InnerText, show in a modal? No, "replace DOM text nodes".
    
    // Let's try to iterate text nodes.
    const walker = document.createTreeWalker(contentDiv, NodeFilter.SHOW_TEXT);
    const nodes: Node[] = [];
    let node;
    while(node = walker.nextNode()) {
        if (node.nodeValue?.trim()) nodes.push(node);
    }

    // This is very slow if we translate node by node.
    // Let's try to grab the whole Markdown container innerText (simulated markdown) if we can.
    
    // Since this is a prototype/demo:
    // We will just create a "Translated View" that might be text-heavy or use the API to return a string.
    // But let's follow the "Translate Page" button instruction.
    
    // ACTUAL IMPLEMENTATION: 
    // We will assume the API accepts HTML content if we send HTML, or we send MD if available.
    // Let's send the innerHTML and ask Gemini to translate the text inside tags.
    
    const html = contentDiv.innerHTML;
    
    try {
        const res = await fetch('/api/translate', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({ content: html, targetLanguage: lang })
        });
        const data = await res.json();
        if (data.translatedContent) {
            contentDiv.innerHTML = data.translatedContent;
        }
    } catch (e) {
        console.error(e);
        alert('Translation failed');
    }
    
    setLoading(false);
  };

  return (
    <div className="flex gap-2 mb-4">
      <button 
        disabled={loading}
        onClick={() => translatePage('ur')}
        className="px-3 py-1 bg-blue-600 text-white rounded hover:bg-blue-700 disabled:opacity-50"
      >
        {loading ? 'Translating...' : 'Translate to Urdu'}
      </button>
       <button 
        disabled={loading}
        onClick={() => translatePage('hi')}
        className="px-3 py-1 bg-orange-600 text-white rounded hover:bg-orange-700 disabled:opacity-50"
      >
        {loading ? 'Translating...' : 'Translate to Hindi'}
      </button>
    </div>
  );
}
