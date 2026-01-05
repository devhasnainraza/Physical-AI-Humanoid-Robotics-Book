import React, { useState } from 'react';
import { useUser } from '@clerk/clerk-react';
import { Languages, Sparkles, Wand2, Loader2 } from 'lucide-react';

export default function ChapterActions() {
  const { isSignedIn, user } = useUser();
  const [loading, setLoading] = useState<string | null>(null);

  if (!isSignedIn) return null;

  const handleAction = async (type: 'translate' | 'personalize') => {
    setLoading(type);
    // Logic: In a real app, this would call an API. 
    // For this prototype, we will trigger the Chatbot to handle it or show a toast.
    setTimeout(() => {
      setLoading(null);
      alert(type === 'translate' ? "Translating chapter to Urdu..." : `Personalizing content for ${user.firstName}...`);
      // We can actually trigger the ChatWidget here if we want to be fancy.
    }, 1500);
  };

  return (
    <div className="flex flex-wrap gap-3 mb-8 p-4 bg-blue-50/50 dark:bg-blue-900/10 border border-blue-100 dark:border-blue-800 rounded-2xl backdrop-blur-sm">
      <div className="w-full mb-2">
        <span className="text-[10px] font-black uppercase tracking-widest text-blue-500 dark:text-blue-400 flex items-center gap-2">
          <Sparkles className="w-3 h-3" /> AI Enhanced Chapter
        </span>
      </div>
      
      <button
        onClick={() => handleAction('translate')}
        disabled={!!loading}
        className="flex items-center gap-2 px-4 py-2.5 bg-white dark:bg-slate-800 border border-blue-200 dark:border-blue-700 rounded-xl text-sm font-bold text-blue-600 dark:text-blue-400 hover:shadow-lg hover:shadow-blue-500/10 transition-all active:scale-95 disabled:opacity-50"
      >
        {loading === 'translate' ? <Loader2 className="w-4 h-4 animate-spin" /> : <Languages className="w-4 h-4" />}
        Translate to Urdu
      </button>

      <button
        onClick={() => handleAction('personalize')}
        disabled={!!loading}
        className="flex items-center gap-2 px-4 py-2.5 bg-gradient-to-r from-blue-600 to-indigo-600 text-white rounded-xl text-sm font-bold hover:shadow-lg hover:shadow-blue-500/20 transition-all active:scale-95 disabled:opacity-50"
      >
        {loading === 'personalize' ? <Loader2 className="w-4 h-4 animate-spin" /> : <Wand2 className="w-4 h-4" />}
        Personalize for Me
      </button>
    </div>
  );
}
