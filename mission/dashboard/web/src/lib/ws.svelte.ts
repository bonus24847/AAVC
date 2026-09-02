// WebSocket client with auto-reconnect, typed dispatch.

import type { WsEnvelope, WsKind } from './types';

type Handler = (payload: unknown) => void;

export class RealtimeClient {
  private socket: WebSocket | null = null;
  private url: string;
  private handlers: Map<WsKind, Set<Handler>> = new Map();
  private reconnectMs = 1500;
  private reconnectTimer: number | null = null;
  public connected = $state(false);

  constructor(url: string) {
    this.url = url;
  }

  start(): void {
    this.openSocket();
  }

  on(kind: WsKind, fn: Handler): void {
    if (!this.handlers.has(kind)) this.handlers.set(kind, new Set());
    this.handlers.get(kind)!.add(fn);
  }

  off(kind: WsKind, fn: Handler): void {
    this.handlers.get(kind)?.delete(fn);
  }

  private openSocket(): void {
    try {
      this.socket = new WebSocket(this.url);
    } catch (e) {
      console.error('[ws] open failed:', e);
      this.scheduleReconnect();
      return;
    }
    this.socket.onopen = () => {
      console.info('[ws] connected:', this.url);
      this.connected = true;
    };
    this.socket.onmessage = (e: MessageEvent) => {
      try {
        const env = JSON.parse(e.data) as WsEnvelope;
        this.dispatch(env);
      } catch (err) {
        console.warn('[ws] parse failed:', err);
      }
    };
    this.socket.onclose = () => {
      this.connected = false;
      this.scheduleReconnect();
    };
    this.socket.onerror = (e: Event) => {
      console.warn('[ws] error:', e);
    };
  }

  private dispatch(env: WsEnvelope): void {
    const fns = this.handlers.get(env.kind);
    if (!fns) return;
    for (const fn of fns) {
      try { fn(env.payload); }
      catch (err) { console.error(`[ws] handler ${env.kind} threw:`, err); }
    }
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer !== null) return;
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.openSocket();
    }, this.reconnectMs);
  }
}
