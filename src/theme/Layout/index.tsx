import React, {type ReactNode, useEffect} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import ChatWidget from '@site/src/components/ChatWidget';
import Translator from '@site/src/components/Translator'; // Import Translator
import ClerkAuth from '@site/src/components/Auth/ClerkAuth';
import { ClerkProvider, SignedIn, SignedOut, RedirectToSignIn } from '@clerk/clerk-react';
import { useLocation } from '@docusaurus/router';

type Props = WrapperProps<typeof LayoutType>;

// Replace with your actual key from Clerk Dashboard
const CLERK_PUBLISHABLE_KEY = 'pk_test_dXByaWdodC1rb2FsYS0zOC5jbGVyay5hY2NvdW50cy5kZXYk'; 

export default function LayoutWrapper(props: Props): ReactNode {
  const location = useLocation();
  const isProtected = location.pathname.includes('/docs/');

  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      if (text && text.length > 0) {
        const event = new CustomEvent('textbook-selection', { detail: text });
        window.dispatchEvent(event);
      }
    };
    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, []);

  return (
    <ClerkProvider publishableKey={CLERK_PUBLISHABLE_KEY}>
      <Layout {...props}>
        <ClerkAuth />
        
        {/* Protected Route Logic */}
        {isProtected ? (
            <>
                <SignedIn>
                    {props.children}
                    <ChatWidget />
                </SignedIn>
                <SignedOut>
                    <div className="flex flex-col items-center justify-center min-h-[60vh] text-center px-4">
                        <h1 className="text-3xl font-bold mb-4">Access Restricted</h1>
                        <p className="mb-6">Please sign in to view the textbook content.</p>
                        <RedirectToSignIn />
                    </div>
                </SignedOut>
            </>
        ) : (
            <>
                {props.children}
                <ChatWidget />
            </>
        )}
      </Layout>
    </ClerkProvider>
  );
}
