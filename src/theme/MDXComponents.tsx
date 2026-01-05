import React from 'react';
// Import the original mapper
import MDXComponents from '@theme-original/MDXComponents';
import HardwareOnly from '@site/src/components/HardwareOnly';
import Translator from '@site/src/components/Translator';
// import ChapterActions from '@site/src/components/ChapterActions';

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Map the "<HardwareOnly>" tag to our component
  HardwareOnly,
  // Override h1 to inject Actions and Translator
  h1: (props) => (
    <header>
      {/* <ChapterActions /> */}
      <h1 {...props} />
      <Translator />
    </header>
  ),
};
