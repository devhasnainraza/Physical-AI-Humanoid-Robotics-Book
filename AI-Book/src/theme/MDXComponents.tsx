import React from 'react';
// Import the original mapper
import MDXComponents from '@theme-original/MDXComponents';
import HardwareOnly from '@site/src/components/HardwareOnly';

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Map the "<HardwareOnly>" tag to our component
  HardwareOnly,
};
