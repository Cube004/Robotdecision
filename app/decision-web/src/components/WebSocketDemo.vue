<template>
  <div class="websocket-demo" :class="{ 'websocket-demo-collapsed': !isExpanded }">
    <div class="demo-header" @click="toggleExpand">
      <h2>WebSocket调试工具</h2>
      <button class="toggle-btn">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"
          :style="{ transform: isExpanded ? 'rotate(0deg)' : 'rotate(180deg)' }">
          <path d="M19 15L12 9L5 15" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" />
        </svg>
      </button>
    </div>

    <div v-if="isExpanded" class="demo-content">
      <div class="connection-form">
        <div class="form-row">
          <div class="form-group">
            <label for="ws-ip">服务器IP:</label>
            <input id="ws-ip" v-model="ip" type="text" placeholder="例如: 192.168.1.100" />
          </div>

          <div class="form-group">
            <label for="ws-port">服务器端口:</label>
            <input id="ws-port" v-model.number="port" type="number" placeholder="例如: 8080" />
          </div>
        </div>

        <div class="button-row">
          <button
            @click="connectWebSocket"
            :disabled="wsConnected"
            class="connect-btn"
          >
            连接
          </button>

          <button
            @click="disconnectWebSocket"
            :disabled="!wsConnected"
            class="disconnect-btn"
          >
            断开连接
          </button>
        </div>
      </div>

      <div class="connection-status">
        <p>连接状态:
          <span :class="{ 'status-connected': wsConnected, 'status-disconnected': !wsConnected }">
            {{ wsConnected ? '已连接' : '未连接' }}
          </span>
        </p>
        <p v-if="wsError" class="error-message">错误信息: {{ wsError }}</p>
      </div>

      <div class="data-section" v-if="wsConnected">
        <h3>接收到的数据</h3>

        <div class="data-tabs">
          <div class="tab-header-container">
            <div class="tab-header">
              <button
                v-for="(tab, index) in dataTabs"
                :key="index"
                :class="{ active: activeTab === index }"
                @click="activeTab = index"
              >
                {{ tab.name }}
                <div class="badge" v-if="getTabItemCount(index) > 0">{{ getTabItemCount(index) }}</div>
              </button>
            </div>
          </div>

          <div class="tab-content">
            <!-- 路径数据 -->
            <div v-if="activeTab === 0">
              <div v-if="pathNodeData.length > 0" class="data-card">
                <h4>路径节点:</h4>
                <div class="path-nodes">
                  <div v-for="(node, index) in pathNodeData" :key="index" class="path-node-item">
                    {{ node }}
                  </div>
                </div>
              </div>
              <div v-else class="empty-state">
                <p>暂无路径数据</p>
              </div>
            </div>

            <!-- 机器人姿态数据 -->
            <div v-if="activeTab === 1">
              <div v-if="poseData.length > 0" class="data-card">
                <h4>机器人姿态:</h4>
                <div class="table-container">
                  <table class="data-table">
                    <thead>
                      <tr>
                        <th>X</th>
                        <th>Y</th>
                        <th>Z</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr v-for="(item, index) in poseData" :key="index">
                        <td>{{ item.x.toFixed(2) }}</td>
                        <td>{{ item.y.toFixed(2) }}</td>
                        <td>{{ item.z.toFixed(2) }}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
              </div>
              <div v-else class="empty-state">
                <p>暂无姿态数据</p>
              </div>
            </div>

            <!-- 其他数据 -->
            <div v-if="activeTab === 2">
              <div v-if="currentDataItems.length > 0" class="data-card">
                <h4>数据项:</h4>
                <div class="search-box">
                  <input v-model="searchQuery" type="text" placeholder="搜索数据项..." />
                </div>
                <div class="table-container">
                  <table class="data-table">
                    <thead>
                      <tr>
                        <th>名称</th>
                        <th>值</th>
                      </tr>
                    </thead>
                    <tbody>
                      <tr v-for="(item, index) in filteredDataItems" :key="index" class="data-row">
                        <td class="data-label">{{ item.label }}</td>
                        <td class="data-value">{{ item.data }}</td>
                      </tr>
                    </tbody>
                  </table>
                </div>
              </div>
              <div v-else class="empty-state">
                <p>暂无数据项</p>
              </div>
            </div>

            <!-- 原始数据 -->
            <div v-if="activeTab === 3">
              <div v-if="rawReceivedData" class="data-card">
                <h4>原始数据:</h4>
                <div class="json-container">
                  <pre>{{ JSON.stringify(rawReceivedData, null, 2) }}</pre>
                </div>
              </div>
              <div v-else class="empty-state">
                <p>暂无原始数据</p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onBeforeUnmount } from 'vue';
import { WebSocketClient, createWebSocketClient, wsConnected, wsError, rawData } from '../services';
import { path_node, current_data, pose } from '../types/extensions/Debug/debug';

// 组件展开/折叠状态
const isExpanded = ref(false);
const toggleExpand = () => {
  isExpanded.value = !isExpanded.value;
};

// WebSocket连接参数
const ip = ref('127.0.0.1');
const port = ref(9000);
let wsClient: WebSocketClient | null = null;

// 搜索功能
const searchQuery = ref('');

// 标签页设置
const activeTab = ref(0);
const dataTabs = [
  { name: '路径节点' },
  { name: '机器人姿态' },
  { name: '数据项' },
  { name: '原始数据' }
];

// 获取各标签页的项目数量
const getTabItemCount = (tabIndex: number) => {
  switch (tabIndex) {
    case 0: return pathNodeData.value.length;
    case 1: return poseData.value.length;
    case 2: return currentDataItems.value.length;
    case 3: return rawReceivedData.value ? 1 : 0;
    default: return 0;
  }
};

// 计算属性
const pathNodeData = computed(() => path_node.value);
const poseData = computed(() => pose.value);
const currentDataItems = computed(() => current_data.value);
const rawReceivedData = computed(() => rawData.value);

// 过滤数据项
const filteredDataItems = computed(() => {
  if (!searchQuery.value.trim()) {
    return currentDataItems.value;
  }
  const query = searchQuery.value.toLowerCase();
  return currentDataItems.value.filter(item =>
    item.label.toLowerCase().includes(query) ||
    String(item.data).toLowerCase().includes(query)
  );
});

// 连接WebSocket
const connectWebSocket = () => {
  if (wsConnected.value || !ip.value || !port.value) {
    return;
  }

  wsClient = createWebSocketClient(ip.value, port.value);
  wsClient.connect();
  isExpanded.value = true; // 连接时展开面板
};

// 断开WebSocket连接
const disconnectWebSocket = () => {
  if (wsClient) {
    wsClient.disconnect();
    wsClient = null;
  }
};

// 组件销毁前断开连接
onBeforeUnmount(() => {
  disconnectWebSocket();
});
</script>

<style scoped>
.websocket-demo {
  position: fixed;
  bottom: 80px;
  left: 20px;
  width: 380px;
  max-width: 90vw;
  background-color: #ffffff;
  border-radius: 10px;
  box-shadow: 0 5px 20px rgba(0, 0, 0, 0.15);
  overflow: hidden;
  transition: all 0.3s ease;
  z-index: 1000;
  max-height: calc(100vh - 140px);
  display: flex;
  flex-direction: column;
}

.websocket-demo-collapsed {
  height: 60px;
  width: 250px;
}

.demo-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 12px 16px;
  background-color: #1e40af;
  color: white;
  cursor: pointer;
  flex-shrink: 0;
}

.demo-header h2 {
  margin: 0;
  font-size: 16px;
  font-weight: 600;
}

.toggle-btn {
  background: transparent;
  border: none;
  color: white;
  cursor: pointer;
  padding: 4px;
  border-radius: 4px;
  display: flex;
  align-items: center;
  justify-content: center;
}

.toggle-btn:hover {
  background-color: rgba(255, 255, 255, 0.1);
}

.demo-content {
  padding: 16px;
  overflow-y: auto;
  flex-grow: 1;
  max-height: calc(100vh - 200px);
}

.connection-form {
  display: flex;
  flex-direction: column;
  gap: 12px;
  margin-bottom: 16px;
  padding: 12px;
  background-color: #f8fafc;
  border-radius: 8px;
  border: 1px solid #e2e8f0;
}

.form-row {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
  width: 100%;
}

.button-row {
  display: flex;
  gap: 8px;
}

.form-group {
  display: flex;
  flex-direction: column;
  flex: 1;
  min-width: 140px;
}

label {
  margin-bottom: 5px;
  font-size: 13px;
  color: #64748b;
  font-weight: 500;
}

input {
  padding: 8px 12px;
  border: 1px solid #cbd5e1;
  border-radius: 6px;
  font-size: 14px;
  transition: border-color 0.2s;
  width: 100%;
  box-sizing: border-box;
}

input:focus {
  border-color: #3b82f6;
  outline: none;
  box-shadow: 0 0 0 3px rgba(59, 130, 246, 0.1);
}

button {
  padding: 8px 16px;
  border: none;
  border-radius: 6px;
  cursor: pointer;
  font-weight: 600;
  font-size: 14px;
  transition: all 0.2s;
}

.connect-btn {
  background-color: #3b82f6;
  color: white;
  flex: 1;
}

.connect-btn:hover:not(:disabled) {
  background-color: #2563eb;
}

.disconnect-btn {
  background-color: #ef4444;
  color: white;
  flex: 1;
}

.disconnect-btn:hover:not(:disabled) {
  background-color: #dc2626;
}

button:disabled {
  background-color: #cbd5e1;
  cursor: not-allowed;
  opacity: 0.7;
}

.connection-status {
  margin-bottom: 16px;
  padding: 12px;
  background-color: #f8fafc;
  border-radius: 8px;
  border: 1px solid #e2e8f0;
}

.status-connected {
  color: #10b981;
  font-weight: bold;
}

.status-disconnected {
  color: #ef4444;
  font-weight: bold;
}

.error-message {
  color: #ef4444;
  margin-top: 6px;
  font-size: 13px;
}

.data-section {
  border-top: 1px solid #e2e8f0;
  padding-top: 16px;
}

h3 {
  margin: 0 0 14px 0;
  font-size: 16px;
  color: #334155;
}

h4 {
  margin: 0 0 10px 0;
  font-size: 14px;
  color: #475569;
  font-weight: 600;
}

.data-tabs {
  margin-top: 16px;
}

.tab-header-container {
  overflow-x: auto;
  margin-bottom: 16px;
}

.tab-header {
  display: flex;
  border-bottom: 1px solid #e2e8f0;
  padding-bottom: 1px;
  min-width: min-content;
}

.tab-header button {
  background: none;
  border: none;
  padding: 8px 16px;
  cursor: pointer;
  border-bottom: 2px solid transparent;
  color: #64748b;
  font-weight: 500;
  white-space: nowrap;
  display: flex;
  align-items: center;
  gap: 6px;
  flex-shrink: 0;
}

.tab-header button.active {
  border-bottom: 2px solid #3b82f6;
  color: #3b82f6;
}

.badge {
  background-color: #dbeafe;
  color: #1e40af;
  border-radius: 10px;
  padding: 1px 6px;
  font-size: 11px;
  min-width: 18px;
  text-align: center;
}

.tab-content {
  padding: 8px 0;
}

.data-card {
  background-color: #ffffff;
  border: 1px solid #e2e8f0;
  border-radius: 8px;
  padding: 16px;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.05);
}

.table-container {
  overflow-x: auto;
  max-width: 100%;
  margin-top: 10px;
}

.data-table {
  width: 100%;
  border-collapse: separate;
  border-spacing: 0;
  border-radius: 8px;
  overflow: hidden;
  min-width: 100%;
}

.data-table th, .data-table td {
  border: 1px solid #e2e8f0;
  padding: 10px 12px;
  text-align: left;
  font-size: 14px;
}

.data-table th {
  background-color: #f1f5f9;
  font-weight: 600;
  color: #334155;
  position: sticky;
  top: 0;
}

.data-table tbody tr:nth-child(even) {
  background-color: #f8fafc;
}

.data-table tbody tr:hover {
  background-color: #f1f5f9;
}

.data-label {
  font-weight: 500;
  color: #334155;
  max-width: 150px;
  overflow: hidden;
  text-overflow: ellipsis;
  word-break: break-all;
}

.data-value {
  font-family: 'Courier New', monospace;
  color: #0f766e;
  max-width: 150px;
  overflow: hidden;
  text-overflow: ellipsis;
  word-break: break-all;
}

.json-container {
  background-color: #0f172a;
  border-radius: 6px;
  overflow: auto;
  max-height: 400px;
  padding: 12px;
}

pre {
  margin: 0;
  color: #e2e8f0;
  font-family: 'Courier New', monospace;
  font-size: 13px;
  white-space: pre-wrap;
  word-break: break-all;
}

.path-nodes {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
}

.path-node-item {
  background-color: #dbeafe;
  color: #1e40af;
  border-radius: 4px;
  padding: 4px 10px;
  font-size: 13px;
  font-weight: 500;
}

.search-box {
  margin-bottom: 14px;
}

.search-box input {
  width: 100%;
  padding: 8px 12px;
  border-radius: 6px;
  border: 1px solid #cbd5e1;
  background-color: #f8fafc;
  font-size: 14px;
}

.empty-state {
  padding: 30px;
  text-align: center;
  color: #94a3b8;
  background-color: #f8fafc;
  border-radius: 8px;
  border: 1px dashed #cbd5e1;
}

@media (max-width: 480px) {
  .websocket-demo {
    max-width: calc(100vw - 40px);
    bottom: 70px;
  }

  .form-row {
    flex-direction: column;
  }

  .button-row {
    flex-direction: column;
  }
}

/* 优化滚动条样式 - 适用于所有可滚动区域 */
.demo-content::-webkit-scrollbar,
.table-container::-webkit-scrollbar,
.json-container::-webkit-scrollbar,
.tab-header-container::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}

.demo-content::-webkit-scrollbar-track,
.table-container::-webkit-scrollbar-track,
.json-container::-webkit-scrollbar-track,
.tab-header-container::-webkit-scrollbar-track {
  background: transparent;
}

.demo-content::-webkit-scrollbar-thumb,
.table-container::-webkit-scrollbar-thumb,
.tab-header-container::-webkit-scrollbar-thumb {
  background-color: rgba(203, 213, 225, 0.5);
  border-radius: 20px;
  transition: background-color 0.3s;
}

.json-container::-webkit-scrollbar-thumb {
  background-color: rgba(226, 232, 240, 0.3);
  border-radius: 20px;
  transition: background-color 0.3s;
}

.demo-content::-webkit-scrollbar-thumb:hover,
.table-container::-webkit-scrollbar-thumb:hover,
.tab-header-container::-webkit-scrollbar-thumb:hover {
  background-color: rgba(148, 163, 184, 0.8);
}

.json-container::-webkit-scrollbar-thumb:hover {
  background-color: rgba(226, 232, 240, 0.5);
}

/* Firefox滚动条样式 */
.demo-content,
.table-container,
.json-container,
.tab-header-container {
  scrollbar-width: thin;
  scrollbar-color: rgba(203, 213, 225, 0.5) transparent;
}

.json-container {
  scrollbar-color: rgba(226, 232, 240, 0.3) transparent;
}
</style>
