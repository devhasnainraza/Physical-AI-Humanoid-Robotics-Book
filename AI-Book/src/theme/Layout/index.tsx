import React, {type ReactNode, useEffect} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import ChatWidget from '@site/src/components/ChatWidget';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  useEffect(() => {
    const handleMouseUp = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();
      
      if (text && text.length > 0) {
        // Dispatch custom event for ChatWidget
        const event = new CustomEvent('textbook-selection', { detail: text });
        window.dispatchEvent(event);
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, []);

  return (
    <AuthProvider>
      <Layout {...props} />
      <ChatWidget />
    </AuthProvider>
  );
}