import { defineConfig } from "vite";
import { viteSingleFile } from "vite-plugin-singlefile";

// One self-contained index.html (scripts, styles, fonts, point clouds inlined) that opens from file://;
// photographs and camera frames stay beside it in dist/img, loaded by relative URL.
export default defineConfig({
  base: "./",
  plugins: [viteSingleFile()],
  build: { chunkSizeWarningLimit: 20000, assetsInlineLimit: 100_000_000 },
});
