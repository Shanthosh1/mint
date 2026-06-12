import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// Dev server proxies API + WS to the FastAPI backend so the SPA is
// origin-agnostic; production build is served by FastAPI itself.
export default defineConfig({
  plugins: [react()],
  server: {
    proxy: {
      '/api': 'http://127.0.0.1:8400',
      '/ws': { target: 'ws://127.0.0.1:8400', ws: true },
    },
  },
  build: { outDir: 'dist', sourcemap: false },
});
