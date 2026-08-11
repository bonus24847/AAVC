import { mount } from 'svelte';
import App from './app.svelte';

// Operational typography — vendored offline for the closed-network
// competition environment. IBM Plex Sans is the UI face (labels,
// buttons, headings); IBM Plex Mono is the data face (telemetry
// numbers, command verbs, log lines). Both ship with tabular figures.
import '@fontsource/ibm-plex-sans/400.css';
import '@fontsource/ibm-plex-sans/500.css';
import '@fontsource/ibm-plex-sans/600.css';
import '@fontsource/ibm-plex-mono/400.css';
import '@fontsource/ibm-plex-mono/600.css';

// MapLibre CSS via npm — works offline (vs the previous unpkg link).
import 'maplibre-gl/dist/maplibre-gl.css';

import './styles/global.css';

mount(App, { target: document.getElementById('app')! });
