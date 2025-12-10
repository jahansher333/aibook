// src/theme/NavbarItem/components.ts

import type { ComponentType } from 'react';
import CustomAuthNavbarItem from './CustomAuthNavbarItem';

// Yeh line sabse zaroori hai
const Components: Record<string, ComponentType<any>> = {
  CustomAuthNavbarItem, // ← Yeh exact name match karega aapke config mein type se
};

export default Components;