/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./app/**/*.{js,ts,jsx,tsx,mdx}",
    "./pages/**/*.{js,ts,jsx,tsx,mdx}",
    "./components/**/*.{js,ts,jsx,tsx,mdx}",
  ],

  theme: {
    extend: {
      colors: {
        // Backgrounds
        window: "#F8F9FA", // gray-100
        background: "#181A18", // soft black
        // Texts
        input: "#000000", // black
        default: "#343A40", // gray-900
        fade: "#868E96", // gray-700
        highlight: "#CFD4DA", // gray-500
        // Validation
        disabled: "#DEE2E6", // gray-400
        valid: "#10B981", // green-500
        invalid: "#EF4444", // red-500

        // Theme
        chrome: "#FB923C",
        rust: "#FDBA74",
        metal: "#FED7AA",

        // Custom Palettes
        gray: {
          100: "#F8F9FA",
          200: "#F1F3F5",
          300: "#E8EBED",
          400: "#DEE2E6",
          500: "#CFD4DA",
          600: "#ADB5BD",
          700: "#868E96",
          800: "#495057",
          900: "#343A40",
        },
      },
    },
  },

  plugins: [],
};
