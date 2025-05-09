import { ref } from 'vue';
import { path_node, current_data, pose } from '../types/extensions/Debug/debug';

// 定义接收到的数据类型
export interface WebSocketData {
  data_tabs?: {
    data?: Record<string, number | string | boolean>;
    robot_pose?: {
      x: number;
      y: number;
      z: number;
    };
    timestamp?: number;
  };
  path?: {
    duration?: number;
    nodeids?: number[];
    target_id?: number;
  };
}

// WebSocket连接状态
export const wsConnected = ref(false);
export const wsError = ref<string | null>(null);

// 原始接收数据
export const rawData = ref<WebSocketData | null>(null);

/**
 * WebSocket客户端类
 */
class WebSocketClient {
  private ws: WebSocket | null = null;
  private url: string;
  private autoReconnect: boolean;
  private reconnectInterval: number;
  private reconnectAttempts: number = 0;
  private maxReconnectAttempts: number;
  private reconnectTimeoutId: number | null = null;

  /**
   * 构造函数
   * @param ip 服务器IP地址
   * @param port 服务器端口
   * @param autoReconnect 是否自动重连
   * @param reconnectInterval 重连间隔(毫秒)
   * @param maxReconnectAttempts 最大重连次数
   */
  constructor(
    ip: string,
    port: number,
    autoReconnect: boolean = true,
    reconnectInterval: number = 3000,
    maxReconnectAttempts: number = 5
  ) {
    this.url = `ws://${ip}:${port}`;
    this.autoReconnect = autoReconnect;
    this.reconnectInterval = reconnectInterval;
    this.maxReconnectAttempts = maxReconnectAttempts;
  }

  /**
   * 连接WebSocket服务器
   */
  connect(): void {
    if (this.ws) {
      this.disconnect();
    }

    try {
      this.ws = new WebSocket(this.url);

      // 连接建立时
      this.ws.onopen = () => {
        console.log('WebSocket连接已建立');
        wsConnected.value = true;
        wsError.value = null;
        this.reconnectAttempts = 0;
      };

      // 接收消息时
      this.ws.onmessage = (event) => {
        try {
          const data = JSON.parse(event.data) as WebSocketData;
          rawData.value = data;
          this.processData(data);
        } catch (err) {
          console.error('解析WebSocket数据失败:', err);
        }
      };

      // 连接关闭时
      this.ws.onclose = () => {
        console.log('WebSocket连接已关闭');
        wsConnected.value = false;

        if (this.autoReconnect && this.reconnectAttempts < this.maxReconnectAttempts) {
          this.scheduleReconnect();
        }
      };

      // 连接错误时
      this.ws.onerror = (error) => {
        console.error('WebSocket错误:', error);
        wsError.value = '连接错误';
        wsConnected.value = false;
      };
    } catch (err) {
      console.error('创建WebSocket连接失败:', err);
      wsError.value = '创建连接失败';
      wsConnected.value = false;

      if (this.autoReconnect && this.reconnectAttempts < this.maxReconnectAttempts) {
        this.scheduleReconnect();
      }
    }
  }

  /**
   * 关闭WebSocket连接
   */
  disconnect(): void {
    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    if (this.reconnectTimeoutId !== null) {
      clearTimeout(this.reconnectTimeoutId);
      this.reconnectTimeoutId = null;
    }

    wsConnected.value = false;
  }

  /**
   * 安排重新连接
   */
  private scheduleReconnect(): void {
    if (this.reconnectTimeoutId !== null) {
      clearTimeout(this.reconnectTimeoutId);
    }

    this.reconnectAttempts++;
    console.log(`尝试重新连接 (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);

    this.reconnectTimeoutId = setTimeout(() => {
      this.connect();
    }, this.reconnectInterval) as unknown as number;
  }

  /**
   * 处理接收到的数据
   * @param data 解析后的JSON数据
   */
  private processData(data: WebSocketData): void {
    try {
      // 处理路径节点数据
      if (data.path && Array.isArray(data.path.nodeids)) {
        path_node.value = data.path.nodeids;
      }

      // 处理机器人姿态数据
      if (data.data_tabs && data.data_tabs.robot_pose) {
        const robotPose = data.data_tabs.robot_pose;
        pose.value = {
          x: robotPose.x || 0,
          y: robotPose.y || 0,
          z: robotPose.z || 0
        };
      }

      // 处理其他数据
      if (data.data_tabs && data.data_tabs.data) {
        const receivedData = data.data_tabs.data;
        const processedData = [];

        // 转换接收到的数据为current_data格式
        for (const [key, value] of Object.entries(receivedData)) {
          if (typeof value === 'number') {
            processedData.push({
              label: key,
              data: value as number
            });
          }
        }

        current_data.value = processedData;
      }
    } catch (err) {
      console.error('处理数据失败:', err);
    }
  }

  /**
   * 发送数据到服务器
   * @param data 要发送的数据
   */
  send(data: string | Record<string, unknown>): void {
    if (this.ws && this.ws.readyState === WebSocket.OPEN) {
      try {
        const jsonStr = typeof data === 'string' ? data : JSON.stringify(data);
        this.ws.send(jsonStr);
      } catch (err) {
        console.error('发送数据失败:', err);
      }
    } else {
      console.warn('WebSocket未连接，无法发送数据');
    }
  }
}

// 创建WebSocket客户端实例的工厂函数
export function createWebSocketClient(
  ip: string,
  port: number,
  autoReconnect: boolean = true,
  reconnectInterval: number = 3000,
  maxReconnectAttempts: number = 5
): WebSocketClient {
  return new WebSocketClient(
    ip,
    port,
    autoReconnect,
    reconnectInterval,
    maxReconnectAttempts
  );
}

export default WebSocketClient;
