import { defineConfig } from 'vite';
import { svelte } from '@sveltejs/vite-plugin-svelte';
import tailwindcss from '@tailwindcss/vite';

const BACKEND_PORT = process.env.AAVC_DASHBOARD_PORT ?? '8765';

export default defineConfig({
  plugins: [svelte(), tailwindcss()],
  base: '/static/',
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    target: 'esnext',
  },
  server: {
    port: 5173,
    strictPort: true,
    proxy: {
      '/api': { target: `http://127.0.0.1:${BACKEND_PORT}`, changeOrigin: true },
      '/ws': { target: `ws://127.0.0.1:${BACKEND_PORT}`, ws: true, changeOrigin: true },
    },
  },
});
