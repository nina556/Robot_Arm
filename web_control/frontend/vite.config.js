import { defineConfig } from 'vite';

export default defineConfig({
  base: '/',
  publicDir: 'public',
  server: {
    proxy: {
      '/api': {
        target: 'http://192.168.10.200:8765',
        changeOrigin: true
      },
      '/snapshot.ply': {
        target: 'http://192.168.10.200:8765',
        changeOrigin: true
      },
      '/vision_stream.mjpg': {
        target: 'http://192.168.10.200:8765',
        changeOrigin: true
      }
    }
  },
  build: {
    outDir: 'dist',
    assetsDir: 'assets',
    emptyOutDir: true,
    chunkSizeWarningLimit: 700
  }
});
