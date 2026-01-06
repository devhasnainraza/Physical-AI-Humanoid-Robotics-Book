// import React, { useState, useEffect } from 'react';
// import ReactDOM from 'react-dom';

// // --- Premium Glass Modal Component for API Key ---
// const ApiKeyModal = ({ isOpen, onSubmit, onClose }: { isOpen: boolean, onSubmit: (key: string) => void, onClose: () => void }) => {
//   const [inputKey, setInputKey] = useState('');
  
//   if (!isOpen) return null;

//   return (
//     <div className="fixed inset-0 z-[300] flex items-center justify-center bg-black/60 backdrop-blur-md animate-fade-in transition-all duration-300">
//       <div className="relative bg-white/90 dark:bg-gray-900/90 backdrop-blur-xl border border-white/20 dark:border-gray-700 p-8 rounded-3xl shadow-2xl max-w-md w-full mx-4 transform transition-all scale-100 hover:shadow-blue-500/10 ring-1 ring-black/5">
        
//         {/* Close Button */}
//         <button onClick={onClose} className="absolute top-4 right-4 p-2 rounded-full text-gray-400 hover:bg-gray-100 dark:hover:bg-gray-800 transition-colors">
//             <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" /></svg>
//         </button>

//         {/* Hero Icon */}
//         <div className="flex justify-center mb-6">
//             <div className="w-16 h-16 bg-blue-50 dark:bg-blue-900/30 rounded-2xl flex items-center justify-center text-3xl shadow-inner text-blue-600 dark:text-blue-400">
//                 🔑
//             </div>
//         </div>

//         <h3 className="text-2xl font-extrabold mb-2 text-center text-gray-900 dark:text-white tracking-tight">
//           Unlock Translation
//         </h3>
//         <p className="text-sm text-center text-gray-500 dark:text-gray-400 mb-6 px-4">
//           Enter your free <b>Google Gemini API Key</b> to enable real-time AI translation for this book.
//         </p>

//         <div className="space-y-4">
//             <div className="relative group">
//                 <input 
//                   type="password" 
//                   value={inputKey}
//                   onChange={(e) => setInputKey(e.target.value)}
//                   placeholder="Paste API Key (starts with AIza...)"
//                   className="w-full p-4 bg-gray-50 dark:bg-black/50 border border-gray-200 dark:border-gray-700 rounded-xl outline-none focus:ring-2 focus:ring-blue-500/50 focus:border-blue-500 transition-all font-mono text-sm shadow-inner group-hover:bg-white dark:group-hover:bg-gray-900"
//                 />
//             </div>

//             <button 
//                 onClick={() => { if(inputKey) onSubmit(inputKey); }}
//                 disabled={!inputKey}
//                 className="w-full py-3.5 bg-gradient-to-r from-blue-600 to-violet-600 hover:from-blue-700 hover:to-violet-700 text-white rounded-xl text-sm font-bold shadow-lg shadow-blue-500/25 disabled:opacity-50 disabled:shadow-none transition-all transform active:scale-[0.98]"
//             >
//                 Start Translating
//             </button>
//         </div>

//         <div className="mt-6 pt-4 border-t border-gray-100 dark:border-gray-800 text-center">
//           <a href="https://aistudio.google.com/app/apikey" target="_blank" rel="noopener noreferrer" className="inline-flex items-center gap-1 text-xs font-semibold text-blue-500 hover:text-blue-600 transition-colors">
//             Get a free key from Google AI Studio
//             <svg className="w-3 h-3" fill="none" viewBox="0 0 24 24" stroke="currentColor"><path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 6H6a2 2 0 00-2 2v10a2 2 0 002 2h10a2 2 0 002-2v-4M14 4h6m0 0v6m0-6L10 14" /></svg>
//           </a>
//         </div>
//       </div>
//     </div>
//   );
// };

// export default function Translator() {
//   const [loading, setLoading] = useState(false);
//   const [currentLang, setCurrentLang] = useState('en');
//   const [originalContent, setOriginalContent] = useState<string | null>(null);
//   const [toast, setToast] = useState<string | null>(null);
//   const [showKeyModal, setShowKeyModal] = useState(false);
//   const [pendingLang, setPendingLang] = useState<'ur'|null>(null);
//   const [navbarContainer, setNavbarContainer] = useState<Element | null>(null);

//   // 1. Initialize: Find Navbar & Save Content
//   useEffect(() => {
//     // Find Docusaurus Navbar Right Container
//     const navRight = document.querySelector('.navbar__items--right');
//     if (navRight) {
//         setNavbarContainer(navRight);
//     }

//     const contentDiv = document.querySelector('.theme-doc-markdown');
//     if (contentDiv && !originalContent) {
//       setOriginalContent(contentDiv.innerHTML);
//     }
//   }, []); 

//   // Premium Visual Loading Effect
//   useEffect(() => {
//     const contentDiv = document.querySelector('.theme-doc-markdown') as HTMLElement;
//     if (contentDiv) {
//       if (loading) {
//         // 1. Dim the original content
//         contentDiv.classList.add('opacity-30', 'pointer-events-none', 'filter', 'blur-[2px]', 'transition-all', 'duration-500');
        
//         // 2. Inject Premium Overlay
//         const overlay = document.createElement('div');
//         overlay.id = 'translator-loading-overlay';
//         // Use FIXED positioning so it stays on screen while scrolling
//         overlay.className = 'fixed inset-0 z-[100] flex flex-col items-center justify-center pointer-events-none';
        
//         overlay.innerHTML = `
//             <div class="relative p-1 rounded-2xl bg-gradient-to-r from-blue-500 via-purple-500 to-pink-500 animate-gradient-xy p-[2px] shadow-2xl pointer-events-auto">
//                 <div class="bg-white dark:bg-gray-900 rounded-xl p-6 flex flex-col items-center gap-4 min-w-[200px]">
                    
//                     <!-- Animated AI Brain Icon -->
//                     <div class="relative w-12 h-12">
//                         <div class="absolute inset-0 bg-blue-500 rounded-full opacity-20 animate-ping"></div>
//                         <div class="relative z-10 w-12 h-12 bg-gradient-to-br from-blue-600 to-violet-600 rounded-full flex items-center justify-center text-white shadow-lg">
//                             <svg class="w-6 h-6 animate-pulse" fill="none" viewBox="0 0 24 24" stroke="currentColor">
//                                 <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 10V3L4 14h7v7l9-11h-7z" />
//                             </svg>
//                         </div>
//                     </div>

//                     <!-- Text & Progress -->
//                     <div class="text-center">
//                         <h3 class="text-lg font-bold bg-clip-text text-transparent bg-gradient-to-r from-blue-600 to-violet-600">Translating...</h3>
//                         <p class="text-xs text-gray-400 mt-1">Generating ${currentLang === 'en' ? 'Urdu' : 'Hindi'} Content</p>
//                     </div>

//                     <!-- Fake Progress Bar -->
//                     <div class="w-full h-1 bg-gray-100 dark:bg-gray-800 rounded-full overflow-hidden">
//                         <div class="h-full bg-blue-500 animate-progress-indeterminate"></div>
//                     </div>
//                 </div>
//             </div>
//         `;
        
//         if (getComputedStyle(contentDiv).position === 'static') {
//             contentDiv.style.position = 'relative';
//         }
//         contentDiv.appendChild(overlay);

//       } else {
//         // Cleanup
//         contentDiv.classList.remove('opacity-30', 'pointer-events-none', 'filter', 'blur-[2px]');
//         const overlay = document.getElementById('translator-loading-overlay');
//         if (overlay) {
//             overlay.classList.add('opacity-0', 'scale-95', 'transition-all', 'duration-300');
//             setTimeout(() => overlay.remove(), 300);
//         }
//         contentDiv.style.position = ''; 
//       }
//     }
//   }, [loading]);

//   const showToast = (msg: string) => {
//     setToast(msg);
//     setTimeout(() => setToast(null), 3000);
//   };

//   const handleKeySubmit = (key: string) => {
//     localStorage.setItem('GEMINI_API_KEY', key);
//     setShowKeyModal(false);
//     if (pendingLang) {
//       translatePage(pendingLang);
//       setPendingLang(null);
//     }
//   };

//   const restoreEnglish = () => {
//     // FORCE RELOAD to ensure 100% clean state
//     window.location.reload();
//   };

//   const translatePage = async (lang: 'ur') => {
//     if (lang as any === 'en') {
//         restoreEnglish();
//         return;
//     }
    
//     if (currentLang === lang) return;

//     let apiKey = localStorage.getItem('GEMINI_API_KEY');
//     if (!apiKey) {
//       setPendingLang(lang);
//       setShowKeyModal(true);
//       return;
//     }

//     setLoading(true);
//     const contentDiv = document.querySelector('.theme-doc-markdown');
//     if (!contentDiv) {
//       setLoading(false);
//       return;
//     }

//     const cacheKey = `trans_cache_${window.location.pathname}_${lang}`;
//     const cachedData = localStorage.getItem(cacheKey);

//     if (cachedData) {
//       setTimeout(() => {
//         contentDiv.innerHTML = cachedData;
//         setCurrentLang(lang);
//         setLoading(false);
//         showToast(`Loaded Urdu (Cached)`);
//       }, 300);
//       return;
//     }

//     try {
//       const { GoogleGenerativeAI } = await import("@google/generative-ai");
//       const genAI = new GoogleGenerativeAI(apiKey);
//       const model = genAI.getGenerativeModel({ model: "gemini-2.5-flash" });
//       const langName = 'Urdu';
      
//       const prompt = `You are a professional technical translator. 
//       Translate the following HTML content to ${langName}.
      
//       CRITICAL RULES:
//       1. Return ONLY the inner HTML content. DO NOT wrap in <html>, <body>, or markdown code blocks.
//       2. DO NOT change any class names, ids, or structure.
//       3. DO NOT add dir="rtl" to the root div (it breaks the site layout).
//       4. DO NOT translate code blocks (<pre>, <code>).
      
//       Content:
//       ${originalContent || contentDiv.innerHTML}`;

//       const result = await model.generateContent(prompt);
//       const response = await result.response;
//       let translatedText = response.text();

//       translatedText = translatedText.replace(/^```html/, '').replace(/```$/, '');

//       if (translatedText) {
//         contentDiv.innerHTML = translatedText;
//         setCurrentLang(lang);
//         localStorage.setItem(cacheKey, translatedText);
//         showToast(`Translated to ${langName}`);
//       }
//     } catch (e: any) {
//       console.error(e);
//       let msg = "Unknown Error";
//       if (e.message?.includes('403') || e.message?.includes('API_KEY_INVALID')) {
//           msg = "Invalid API Key. Please check permissions.";
//           localStorage.removeItem('GEMINI_API_KEY'); 
//       } else if (e.message?.includes('429')) {
//           msg = "Quota Exceeded. Please wait a moment.";
//       } else if (e.message?.includes('404')) {
//           msg = "Model Error. Please contact support.";
//       }
//       alert(`Translation Error: ${msg}`);
//     }
    
//     setLoading(false);
//   };

//   // Premium Dropdown Style with Native Docusaurus Theme Integration
//   const TranslatorControls = () => {
//     const [isOpen, setIsOpen] = useState(false);

//     // Close dropdown on click outside
//     useEffect(() => {
//         const close = () => setIsOpen(false);
//         if (isOpen) window.addEventListener('click', close);
//         return () => window.removeEventListener('click', close);
//     }, [isOpen]);

//     return (
//         <div className="relative mx-1 sm:mx-2" onClick={(e) => e.stopPropagation()}>
//             <button 
//                 onClick={() => setIsOpen(!isOpen)}
//                 className={`relative flex items-center gap-1 sm:gap-2 px-2 sm:px-3 py-1.5 sm:py-2 rounded-xl transition-all duration-300 group overflow-hidden
//                     ${isOpen 
//                         ? 'bg-emerald-500/10 text-emerald-600 dark:text-emerald-400 ring-2 ring-emerald-500/20' 
//                         : 'bg-transparent hover:bg-slate-100 dark:hover:bg-slate-800 text-slate-600 dark:text-slate-300'
//                     }`}
//             >
//                 {/* Gradient Border Effect */}
//                 <div className={`absolute inset-0 rounded-xl border border-transparent transition-colors duration-300 ${isOpen ? 'border-emerald-500/30' : 'group-hover:border-slate-300 dark:group-hover:border-slate-600'}`}></div>

//                 <span className="text-xl filter drop-shadow-sm group-hover:scale-110 transition-transform duration-300 relative z-10">🌐</span>
//                 <span className="text-sm font-bold hidden sm:block relative z-10">
//                     {currentLang === 'en' ? 'English' : 'Urdu'}
//                 </span>
//                 <svg className={`w-4 h-4 transition-transform duration-300 hidden sm:block relative z-10 ${isOpen ? 'rotate-180 text-emerald-500' : ''}`} fill="none" viewBox="0 0 24 24" stroke="currentColor">
//                     <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
//                 </svg>
//             </button>

//             {/* Dropdown Menu - Glassmorphism */}
//             <div className={`absolute right-0 top-full mt-2 min-w-[160px] backdrop-blur-xl bg-white/90 dark:bg-slate-900/90 rounded-2xl shadow-xl border border-slate-200/50 dark:border-slate-700/50 overflow-hidden transition-all duration-300 origin-top-right z-[100]
//                 ${isOpen ? 'opacity-100 visible translate-y-0 scale-100' : 'opacity-0 invisible -translate-y-2 scale-95'}
//             `}>
//                 <div className="p-1.5">
//                     <button 
//                         onClick={() => { restoreEnglish(); setIsOpen(false); }}
//                         className={`w-full text-left flex items-center gap-3 px-3 py-2.5 rounded-xl text-sm font-medium transition-all duration-200
//                             ${currentLang === 'en' 
//                                 ? 'bg-emerald-50 text-emerald-700 dark:bg-emerald-900/20 dark:text-emerald-400' 
//                                 : 'text-slate-600 dark:text-slate-300 hover:bg-slate-100 dark:hover:bg-slate-800'
//                             }
//                         `}
//                     >
//                         <span className="text-lg leading-none filter drop-shadow-sm">🇺🇸</span>
//                         <span>English</span>
//                         {currentLang === 'en' && <div className="ml-auto w-1.5 h-1.5 rounded-full bg-emerald-500"></div>}
//                     </button>

//                     <button 
//                         disabled={loading}
//                         onClick={() => { translatePage('ur'); setIsOpen(false); }}
//                         className={`w-full text-left flex items-center gap-3 px-3 py-2.5 rounded-xl text-sm font-medium transition-all duration-200 mt-1
//                             ${currentLang === 'ur' 
//                                 ? 'bg-emerald-50 text-emerald-700 dark:bg-emerald-900/20 dark:text-emerald-400' 
//                                 : 'text-slate-600 dark:text-slate-300 hover:bg-slate-100 dark:hover:bg-slate-800'
//                             }
//                         `}
//                     >
//                         {loading && currentLang !== 'ur' ? (
//                             <div className="w-5 h-5 border-2 border-emerald-500 border-t-transparent rounded-full animate-spin"></div>
//                         ) : (
//                             <span className="text-lg leading-none filter drop-shadow-sm">🇵🇰</span>
//                         )}
//                         <span>Urdu</span>
//                         {currentLang === 'ur' && <div className="ml-auto w-1.5 h-1.5 rounded-full bg-emerald-500"></div>}
//                     </button>
//                 </div>
//             </div>
//         </div>
//     );
//   };

//   // Use Portal to inject into Navbar if available
//   if (navbarContainer) {
//       return (
//         <>
//             {ReactDOM.createPortal(<TranslatorControls />, navbarContainer)}
//             <ApiKeyModal isOpen={showKeyModal} onSubmit={handleKeySubmit} onClose={() => setShowKeyModal(false)} />
//             {/* Toast remains floating */}
//             <div className={`fixed top-24 left-1/2 transform -translate-x-1/2 z-[300] transition-all duration-300 ${toast ? 'opacity-100 translate-y-0' : 'opacity-0 -translate-y-4 pointer-events-none'}`}>
//                 <div className="bg-gray-900/95 backdrop-blur-md text-white px-6 py-2.5 rounded-full shadow-2xl flex items-center gap-3 border border-white/10 font-medium text-sm">
//                 <span className="text-lg">✨</span>
//                 <span>{toast}</span>
//                 </div>
//             </div>
//         </>
//       );
//   }

//   // Fallback if Navbar not found (e.g. initial load)
//   return null;
// }







import React, { useState, useEffect } from 'react';
import ReactDOM from 'react-dom';

// --- Premium Glass Modal Component for API Key ---
const ApiKeyModal = ({ isOpen, onSubmit, onClose }: { isOpen: boolean; onSubmit: (key: string) => void; onClose: () => void }) => {
  const [inputKey, setInputKey] = useState('');

  if (!isOpen) return null;

  return (
    <div className="fixed inset-0 z-[300] flex items-center justify-center bg-black/50 backdrop-blur-sm animate-fade-in">
      <div className="relative bg-white dark:bg-gray-800/95 backdrop-blur-xl border border-gray-200 dark:border-gray-700/60 p-6 sm:p-8 rounded-2xl shadow-2xl max-w-md w-full mx-4 ring-1 ring-black/5 dark:ring-white/5">
        
        {/* Close Button */}
        <button
          onClick={onClose}
          className="absolute top-3.5 right-3.5 p-1.5 rounded-lg text-gray-500 hover:bg-gray-100 dark:hover:bg-gray-700/50 hover:text-gray-700 dark:hover:text-gray-300 transition-colors"
          aria-label="Close"
        >
          <svg className="w-5 h-5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
          </svg>
        </button>

        {/* Hero Icon */}
        <div className="flex justify-center mb-5">
          <div className="w-14 h-14 bg-blue-50 dark:bg-blue-900/20 rounded-xl flex items-center justify-center text-2xl shadow-sm text-blue-600 dark:text-blue-400">
            🔑
          </div>
        </div>

        <h3 className="text-xl sm:text-2xl font-bold mb-2 text-center text-gray-900 dark:text-white">
          Unlock Translation
        </h3>
        <p className="text-sm text-center text-gray-600 dark:text-gray-300 mb-5 px-2">
          Enter your free <span className="font-semibold">Google Gemini API Key</span> to enable real-time AI translation.
        </p>

        <div className="space-y-4">
          <div className="relative">
            <input
              type="password"
              value={inputKey}
              onChange={(e) => setInputKey(e.target.value)}
              placeholder="Paste API Key (starts with AIza...)"
              className="w-full px-4 py-3 bg-gray-50 dark:bg-gray-900/50 border border-gray-300 dark:border-gray-600 rounded-xl outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500 transition-colors font-mono text-sm placeholder:text-gray-400 dark:placeholder:text-gray-500"
            />
          </div>

          <button
            onClick={() => inputKey && onSubmit(inputKey)}
            disabled={!inputKey}
            className="w-full py-3 bg-gradient-to-r from-blue-600 to-violet-600 hover:from-blue-700 hover:to-violet-700 text-white rounded-xl text-sm font-semibold shadow-lg shadow-blue-500/20 disabled:opacity-60 disabled:cursor-not-allowed transition-all active:scale-[0.98]"
          >
            Start Translating
          </button>
        </div>

        <div className="mt-6 pt-5 border-t border-gray-200 dark:border-gray-700/50 text-center">
          <a
            href="https://aistudio.google.com/app/apikey"
            target="_blank"
            rel="noopener noreferrer"
            className="inline-flex items-center gap-1.5 text-sm font-medium text-blue-600 hover:text-blue-700 dark:text-blue-400 dark:hover:text-blue-300 transition-colors"
          >
            Get a free key from Google AI Studio
            <svg className="w-3.5 h-3.5" fill="none" viewBox="0 0 24 24" stroke="currentColor">
              <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 6H6a2 2 0 00-2 2v10a2 2 0 002 2h10a2 2 0 002-2v-4M14 4h6m0 0v6m0-6L10 14" />
            </svg>
          </a>
        </div>
      </div>
    </div>
  );
};

export default function Translator() {
  const [loading, setLoading] = useState(false);
  const [currentLang, setCurrentLang] = useState('en');
  const [originalContent, setOriginalContent] = useState<string | null>(null);
  const [toast, setToast] = useState<string | null>(null);
  const [showKeyModal, setShowKeyModal] = useState(false);
  const [pendingLang, setPendingLang] = useState<'ur' | null>(null);
  const [navbarContainer, setNavbarContainer] = useState<Element | null>(null);

  // 1. Initialize: Find Navbar & Save Content
  useEffect(() => {
    const navRight = document.querySelector('.navbar__items--right');
    if (navRight) setNavbarContainer(navRight);

    const contentDiv = document.querySelector('.theme-doc-markdown');
    if (contentDiv && !originalContent) {
      setOriginalContent(contentDiv.innerHTML);
    }
  }, [originalContent]);

  // Premium Visual Loading Effect
  useEffect(() => {
    const contentDiv = document.querySelector('.theme-doc-markdown') as HTMLElement;
    if (!contentDiv) return;

    if (loading) {
      contentDiv.classList.add('opacity-30', 'pointer-events-none', 'blur-sm', 'transition-all', 'duration-500');
      if (getComputedStyle(contentDiv).position === 'static') {
        contentDiv.style.position = 'relative';
      }

      const overlay = document.createElement('div');
      overlay.id = 'translator-loading-overlay';
      overlay.className = 'fixed inset-0 z-[100] flex items-center justify-center pointer-events-auto';
      overlay.innerHTML = `
        <div class="relative p-1 rounded-2xl bg-gradient-to-r from-blue-500 via-purple-500 to-pink-500 animate-gradient-xy">
          <div class="bg-white dark:bg-gray-800 rounded-xl p-5 sm:p-6 flex flex-col items-center gap-3 min-w-[220px] shadow-xl">
            <div class="relative w-12 h-12">
              <div class="absolute inset-0 bg-blue-500 rounded-full opacity-20 animate-ping"></div>
              <div class="relative z-10 w-12 h-12 bg-gradient-to-br from-blue-600 to-violet-600 rounded-full flex items-center justify-center text-white shadow-md">
                <svg class="w-6 h-6 animate-pulse" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                  <path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
            </div>
            <div class="text-center">
              <h3 class="text-lg font-bold text-gray-800 dark:text-white">Translating...</h3>
              <p class="text-xs text-gray-500 dark:text-gray-400 mt-1">Generating ${currentLang === 'en' ? 'Urdu' : 'Hindi'} Content</p>
            </div>
            <div class="w-full h-1.5 bg-gray-200 dark:bg-gray-700 rounded-full overflow-hidden">
              <div class="h-full bg-blue-500 animate-progress-indeterminate w-3/4"></div>
            </div>
          </div>
        </div>
      `;
      document.body.appendChild(overlay);
    } else {
      contentDiv.classList.remove('opacity-30', 'pointer-events-none', 'blur-sm');
      contentDiv.style.position = '';
      const overlay = document.getElementById('translator-loading-overlay');
      if (overlay) {
        overlay.classList.add('opacity-0');
        setTimeout(() => overlay.remove(), 300);
      }
    }
  }, [loading, currentLang]);

  const showToast = (msg: string) => {
    setToast(msg);
    setTimeout(() => setToast(null), 3000);
  };

  const handleKeySubmit = (key: string) => {
    localStorage.setItem('GEMINI_API_KEY', key);
    setShowKeyModal(false);
    if (pendingLang) {
      translatePage(pendingLang);
      setPendingLang(null);
    }
  };

  const restoreEnglish = () => {
    window.location.reload();
  };

  const translatePage = async (lang: 'ur') => {
    if (lang as any === 'en') {
      restoreEnglish();
      return;
    }

    if (currentLang === lang) return;

    let apiKey = localStorage.getItem('GEMINI_API_KEY');
    if (!apiKey) {
      setPendingLang(lang);
      setShowKeyModal(true);
      return;
    }

    setLoading(true);
    const contentDiv = document.querySelector('.theme-doc-markdown');
    if (!contentDiv) {
      setLoading(false);
      return;
    }

    const cacheKey = `trans_cache_${window.location.pathname}_${lang}`;
    const cachedData = localStorage.getItem(cacheKey);

    if (cachedData) {
      setTimeout(() => {
        contentDiv.innerHTML = cachedData;
        setCurrentLang(lang);
        setLoading(false);
        showToast(`Loaded Urdu (Cached)`);
      }, 300);
      return;
    }

    try {
      // Connect to Backend
      const BACKEND_URL = "https://cortex-h1.vercel.app/api/py"; 
      
      const response = await fetch(`${BACKEND_URL}/translate`, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          content: originalContent || contentDiv.innerHTML,
          target_language: lang,
          api_key: apiKey // Send user's key to backend
        }),
      });

      if (!response.ok) {
        const errData = await response.json();
        throw new Error(errData.detail || "Translation failed");
      }

      const data = await response.json();
      let translatedText = data.translated_text;

      if (translatedText) {
        contentDiv.innerHTML = translatedText;
        setCurrentLang(lang);
        localStorage.setItem(cacheKey, translatedText);
        showToast(`Translated to Urdu`);
      }
    } catch (e: any) {
      console.error(e);
      let msg = "Unknown Error";
      if (e.message?.includes('403') || e.message?.includes('API_KEY_INVALID')) {
        msg = "Invalid API Key. Re-enter it.";
        localStorage.removeItem('GEMINI_API_KEY');
      } else if (e.message?.includes('429')) {
        msg = "Quota Exceeded. Try again soon.";
      } else if (e.message?.includes('404')) {
        msg = "Model Error. Contact support.";
      }
      alert(`Translation Error: ${msg}`);
    }
    setLoading(false);
  };

  const TranslatorControls = () => {
    const [isOpen, setIsOpen] = useState(false);

    useEffect(() => {
      const close = () => setIsOpen(false);
      if (isOpen) window.addEventListener('click', close);
      return () => window.removeEventListener('click', close);
    }, [isOpen]);

    return (
      <div className="relative mx-1 sm:mx-2" onClick={(e) => e.stopPropagation()}>
        <button
          onClick={() => setIsOpen(!isOpen)}
          className={`relative flex items-center gap-1 sm:gap-2 px-2.5 sm:px-3 py-1.5 sm:py-2 rounded-xl transition-colors duration-200 group
            ${isOpen
                ? 'text-emerald-600 dark:text-emerald-400 bg-emerald-50 dark:bg-emerald-900/20 ring-1 ring-emerald-500/30'
                : 'text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700/50'
            }`}
        >
          <span className="text-xl">🌐</span>
          <span className="text-sm font-medium hidden sm:block">{
            currentLang === 'en' ? 'English' : 'Urdu'
          }</span>
          <svg
            className={`w-4 h-4 transition-transform hidden sm:block ${isOpen ? 'rotate-180 text-emerald-500' : 'text-gray-500 dark:text-gray-400'}`}
            fill="none"
            viewBox="0 0 24 24"
            stroke="currentColor"
          >
            <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 9l-7 7-7-7" />
          </svg>
        </button>

        <div
          className={`absolute right-0 top-full mt-2 min-w-[160px] bg-white dark:bg-gray-800 backdrop-blur-xl rounded-xl shadow-lg border border-gray-200 dark:border-gray-700/50 overflow-hidden transition-all duration-200 origin-top-right z-[100]
            ${isOpen ? 'opacity-100 scale-100 visible' : 'opacity-0 scale-95 invisible'}`}
        >
          <div className="p-1">
            <button
              onClick={() => { restoreEnglish(); setIsOpen(false); }}
              className={`w-full text-left flex items-center gap-3 px-3 py-2.5 rounded-lg text-sm font-medium
                ${currentLang === 'en'
                    ? 'bg-emerald-50 text-emerald-700 dark:bg-emerald-900/30 dark:text-emerald-400'
                    : 'text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700/50'
                }`}
            >
              <span className="text-lg">🇺🇸</span>
              <span>English</span>
              {currentLang === 'en' && <span className="ml-auto w-2 h-2 rounded-full bg-emerald-500" />}
            </button>

            <button
              disabled={loading}
              onClick={() => { translatePage('ur'); setIsOpen(false); }}
              className={`w-full text-left flex items-center gap-3 px-3 py-2.5 rounded-lg text-sm font-medium mt-1
                ${currentLang === 'ur'
                    ? 'bg-emerald-50 text-emerald-700 dark:bg-emerald-900/30 dark:text-emerald-400'
                    : 'text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700/50'
                } ${loading && currentLang !== 'ur' ? 'opacity-80 cursor-not-allowed' : ''}`}
            >
              {loading && currentLang !== 'ur' ? (
                <div className="w-5 h-5 border-2 border-emerald-500 border-t-transparent rounded-full animate-spin" />
              ) : (
                <span className="text-lg">🇵🇰</span>
              )}
              <span>Urdu</span>
              {currentLang === 'ur' && <span className="ml-auto w-2 h-2 rounded-full bg-emerald-500" />}
            </button>
          </div>
        </div>
      </div>
    );
  };

  if (navbarContainer) {
    return (
      <>
        {ReactDOM.createPortal(<TranslatorControls />, navbarContainer)}
        <ApiKeyModal isOpen={showKeyModal} onSubmit={handleKeySubmit} onClose={() => setShowKeyModal(false)} />
        
        {/* Toast */}
        {toast && (
          <div className="fixed top-24 left-1/2 transform -translate-x-1/2 z-[300] animate-fade-in">
            <div className="bg-gray-900/90 backdrop-blur-md text-white px-5 py-2 rounded-full shadow-xl flex items-center gap-2.5 border border-white/10 font-medium text-sm min-w-max">
              <span>✨</span>
              <span>{toast}</span>
            </div>
          </div>
        )}
      </>
    );
  }

  return null;
}