<template>
  <div class="websocket-demo" :class="{ 'websocket-demo-collapsed': !isExpanded }">
    <div class="demo-header" @click="toggleExpand">
      <h2>WebSocket调试工具</h2>
      <button class="toggle-btn" v-if="false">
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg"
          :style="{ transform: isExpanded ? 'rotate(0deg)' : 'rotate(180deg)' }">
          <path d="M19 15L12 9L5 15" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" />
        </svg>
      </button>
    </div>

    <div v-if="isExpanded || true" class="demo-content">
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
              <div v-if="poseData" class="data-card">
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
                      <tr>
                        <td>{{ poseData.x.toFixed(2) }}</td>
                        <td>{{ poseData.y.toFixed(2) }}</td>
                        <td>{{ poseData.z.toFixed(2) }}</td>
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

            <!-- 推送数据 -->
            <div v-if="activeTab === 4">
              <div class="data-card push-data-card">
                <h4>推送数据设置</h4>

                <div class="send-control-panel">
                  <div class="form-row frequency-row">
                    <div class="form-group">
                      <label for="send-frequency">发送频率 (毫秒):</label>
                      <input
                        id="send-frequency"
                        v-model.number="sendFrequency"
                        type="number"
                        min="100"
                        step="100"
                        :disabled="isAutoSending"
                        placeholder="例如: 1000"
                      />
                    </div>

                    <div class="auto-send-controls">
                      <button
                        v-if="!isAutoSending"
                        @click="startAutoSend"
                        :disabled="!wsConnected || !sendFrequency"
                        class="start-btn"
                      >
                        开始发送
                      </button>
                      <button
                        v-else
                        @click="stopAutoSend"
                        class="stop-btn"
                      >
                        停止发送
                      </button>
                    </div>
                  </div>

                  <div class="send-status" v-if="isAutoSending">
                    <div class="status-indicator"></div>
                    <span>每 {{ sendFrequency }}ms 自动发送中 (已发送 {{ sendCount }} 次)</span>
                  </div>
                </div>

                <div class="data-preview">
                  <h5>发送数据预览:</h5>
                  <div class="json-preview">
                    <pre>{{ JSON.stringify(pushDataItems, null, 2) }}</pre>
                  </div>
                </div>

                <div class="edit-push-data">
                  <h5>编辑推送数据:</h5>
                  <div class="search-box">
                    <input v-model="pushSearchQuery" type="text" placeholder="搜索数据项..." />
                  </div>

                  <div class="table-container">
                    <table class="data-table">
                      <thead>
                        <tr>
                          <th>名称</th>
                          <th>值</th>
                          <th>操作</th>
                        </tr>
                      </thead>
                      <tbody>
                        <tr v-for="(item, index) in filteredPushDataItems" :key="index" class="data-row">
                          <td class="data-label">{{ item.label }}</td>
                          <td class="data-value-edit">
                            <input
                              v-model.number="filteredPushDataItems[index].data"
                              type="number"
                              class="value-input"
                            />
                          </td>
                          <td class="data-actions">
                            <button @click="resetPushDataItem(item)" class="reset-btn">重置</button>
                          </td>
                        </tr>
                      </tbody>
                    </table>
                  </div>

                  <div class="button-row push-data-actions">
                    <button
                      @click="sendPushDataOnce"
                      :disabled="!wsConnected"
                      class="send-once-btn"
                    >
                      立即发送一次
                    </button>
                    <button
                      @click="resetAllPushData"
                      class="reset-all-btn"
                    >
                      重置所有数据
                    </button>
                  </div>
                </div>
              </div>
            </div>

            <!-- 规则上传 -->
            <div v-if="activeTab === 5">
              <div class="data-card rules-card">
                <h4>规则上传</h4>

                <div class="rules-info">
                  <p>点击下方按钮将当前规则转换为字符串并发送至服务器。</p>
                  <p class="rules-note">注意：此操作将获取当前编辑器中的所有规则数据，包括节点、边缘、区域等配置。</p>
                </div>

                <div class="rule-preview" v-if="rulePreview">
                  <h5>规则预览:</h5>
                  <div class="json-preview rules-json-preview">
                    <pre>{{ rulePreviewFormatted }}</pre>
                  </div>
                  <div class="rules-file-info">
                    <span>规则大小: {{ ruleSizeFormatted }}</span>
                  </div>
                </div>

                <div class="rules-actions">
                  <button
                    @click="previewRule"
                    class="preview-btn"
                  >
                    生成规则
                  </button>
                  <button
                    @click="sendRule"
                    :disabled="!wsConnected || !rulePreview"
                    class="send-rules-btn"
                  >
                    发送规则
                  </button>
                </div>

                <div v-if="rulesSent" class="rules-sent-info">
                  <div class="success-icon">✓</div>
                  <span>规则已成功发送！</span>
                  <span class="timestamp">{{ ruleSentTimestamp }}</span>
                </div>
              </div>
            </div>

          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onBeforeUnmount, onMounted } from 'vue';
import { WebSocketClient, createWebSocketClient, wsConnected, wsError, rawData } from '../services';
import { path_node, current_data, pose, push_data } from '../types/extensions/Debug/debug';
import { GetRule } from '../types/extensions/MangerTool/Export';

// 组件展开/折叠状态
const isExpanded = ref(true);
const toggleExpand = () => {
  isExpanded.value = !isExpanded.value;
};

// WebSocket连接参数
const ip = ref('172.20.0.203');
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
  { name: '原始数据' },
  { name: '推送数据' },
  { name: '规则上传' }
];

// 获取各标签页的项目数量
const getTabItemCount = (tabIndex: number) => {
  switch (tabIndex) {
    case 0: return pathNodeData.value.length;
    case 1: return poseData.value ? 1 : 0;
    case 2: return currentDataItems.value.length;
    case 3: return rawReceivedData.value ? 1 : 0;
    case 4: return pushDataItems.value.length;
    case 5: return rulePreview.value ? 1 : 0;
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

  // 确保停止自动发送
  stopAutoSend();
};

// 组件销毁前断开连接
onBeforeUnmount(() => {
  disconnectWebSocket();
});

// 推送数据
interface PushDataItem {
  label: string;
  data: number;
}

const pushDataItems = ref<PushDataItem[]>([]);
const pushSearchQuery = ref('');
const sendTimer = ref<number | null>(null);

// 初始化推送数据
onMounted(() => {
  // 复制push_data的初始值
  pushDataItems.value = push_data.value.map(item => ({...item}));
});

const filteredPushDataItems = computed(() => {
  if (!pushSearchQuery.value.trim()) {
    return pushDataItems.value;
  }
  const query = pushSearchQuery.value.toLowerCase();
  return pushDataItems.value.filter(item =>
    item.label.toLowerCase().includes(query) ||
    String(item.data).toLowerCase().includes(query)
  );
});

// 立即发送一次数据
const sendPushDataOnce = () => {
  if (wsClient && pushDataItems.value.length > 0) {
    const jsonData = JSON.stringify(pushDataItems.value);
    wsClient.send(jsonData);
    console.log('发送数据:', jsonData);
  }
};

// 重置单个数据项
const resetPushDataItem = (item: PushDataItem) => {
  const originalItem = push_data.value.find(i => i.label === item.label);
  if (originalItem) {
    const index = pushDataItems.value.findIndex(i => i.label === item.label);
    if (index !== -1) {
      pushDataItems.value[index].data = originalItem.data;
    }
  }
};

// 重置所有数据
const resetAllPushData = () => {
  pushDataItems.value = push_data.value.map(item => ({...item}));
};

// 自动发送控制
const sendFrequency = ref(1000);
const isAutoSending = ref(false);
const sendCount = ref(0);

// 开始自动发送
const startAutoSend = () => {
  if (!wsClient || pushDataItems.value.length === 0) return;

  isAutoSending.value = true;
  sendCount.value = 0;
  autoSend();
};

// 停止自动发送
const stopAutoSend = () => {
  isAutoSending.value = false;
  if (sendTimer.value !== null) {
    window.clearTimeout(sendTimer.value);
    sendTimer.value = null;
  }
};

// 自动发送函数
const autoSend = () => {
  if (isAutoSending.value && wsClient && pushDataItems.value.length > 0) {
    const jsonData = JSON.stringify(pushDataItems.value);
    wsClient.send(jsonData);
    sendCount.value++;

    // 如果还在自动发送状态，设置下一次发送
    if (isAutoSending.value) {
      sendTimer.value = window.setTimeout(autoSend, sendFrequency.value) as unknown as number;
    }
  } else {
    isAutoSending.value = false;
  }
};

// 规则上传功能
const rulePreview = ref<object | null>(null);
const rulesSent = ref(false);
const ruleSentTimestamp = ref('');

// 规则预览
const previewRule = () => {
  try {
    const rule = GetRule();
    rulePreview.value = rule;
    rulesSent.value = false;
  } catch (error) {
    console.error('获取规则时出错:', error);
  }
};

// 获取格式化的JSON字符串
const rulePreviewFormatted = computed(() => {
  if (!rulePreview.value) return '';
  // 格式化但限制显示长度，避免界面过大
  const jsonStr = JSON.stringify(rulePreview.value, null, 2);
  if (jsonStr.length > 1000) {
    return jsonStr.substring(0, 1000) + '...\n[内容过长，仅显示部分]';
  }
  return jsonStr;
});

// 获取规则大小
const ruleSizeFormatted = computed(() => {
  if (!rulePreview.value) return '0 KB';
  const jsonStr = JSON.stringify(rulePreview.value);
  const bytes = new TextEncoder().encode(jsonStr).length;
  if (bytes < 1024) {
    return `${bytes} B`;
  } else if (bytes < 1024 * 1024) {
    return `${(bytes / 1024).toFixed(2)} KB`;
  } else {
    return `${(bytes / (1024 * 1024)).toFixed(2)} MB`;
  }
});

// 发送规则
const sendRule = () => {
  if (wsClient && rulePreview.value) {
    try {
      const ruleString = JSON.stringify(rulePreview.value);
      wsClient.send(ruleString);

      // 更新发送状态
      rulesSent.value = true;
      const now = new Date();
      ruleSentTimestamp.value = `${now.toLocaleDateString()} ${now.toLocaleTimeString()}`;

      console.log('规则已发送:', rulePreview.value);
    } catch (error) {
      console.error('发送规则时出错:', error);
    }
  }
};
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

/* 推送数据样式 */
.push-data-card {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.send-control-panel {
  background-color: #f1f5f9;
  padding: 12px;
  border-radius: 6px;
  margin-bottom: 8px;
}

.frequency-row {
  display: flex;
  align-items: flex-end;
  gap: 12px;
}

.auto-send-controls {
  display: flex;
  gap: 8px;
}

.start-btn {
  background-color: #10b981;
  color: white;
}

.start-btn:hover:not(:disabled) {
  background-color: #059669;
}

.stop-btn {
  background-color: #f97316;
  color: white;
}

.stop-btn:hover {
  background-color: #ea580c;
}

.send-status {
  display: flex;
  align-items: center;
  gap: 8px;
  margin-top: 12px;
  font-size: 13px;
  color: #334155;
}

.status-indicator {
  width: 10px;
  height: 10px;
  border-radius: 50%;
  background-color: #10b981;
  animation: pulse 1s infinite;
}

@keyframes pulse {
  0% {
    opacity: 1;
    transform: scale(1);
  }
  50% {
    opacity: 0.5;
    transform: scale(1.1);
  }
  100% {
    opacity: 1;
    transform: scale(1);
  }
}

.data-preview {
  background-color: #f8fafc;
  padding: 12px;
  border-radius: 6px;
}

.data-preview h5 {
  margin: 0 0 8px 0;
  font-size: 13px;
  color: #64748b;
  font-weight: 500;
}

.json-preview {
  background-color: #f1f5f9;
  border-radius: 4px;
  padding: 8px;
  max-height: 120px;
  overflow: auto;
}

.json-preview pre {
  margin: 0;
  font-size: 12px;
  white-space: pre-wrap;
  color: #334155;
}

.edit-push-data h5 {
  margin: 0 0 12px 0;
  font-size: 13px;
  color: #64748b;
  font-weight: 500;
}

.data-value-edit {
  padding: 4px 0;
}

.value-input {
  width: 100%;
  padding: 6px 8px;
  border: 1px solid #cbd5e1;
  border-radius: 4px;
  font-size: 13px;
  background-color: #f8fafc;
}

.data-actions {
  text-align: center;
  width: 80px;
}

.reset-btn {
  padding: 4px 8px;
  font-size: 12px;
  color: #64748b;
  background-color: #f1f5f9;
  border-radius: 4px;
}

.reset-btn:hover {
  background-color: #e2e8f0;
  color: #334155;
}

.push-data-actions {
  margin-top: 16px;
  display: flex;
  justify-content: space-between;
}

.send-once-btn {
  background-color: #3b82f6;
  color: white;
}

.send-once-btn:hover:not(:disabled) {
  background-color: #2563eb;
}

.reset-all-btn {
  background-color: #94a3b8;
  color: white;
}

.reset-all-btn:hover {
  background-color: #64748b;
}

/* 规则上传样式 */
.rules-card {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.rules-info {
  background-color: #f8fafc;
  padding: 12px;
  border-radius: 6px;
  margin-bottom: 8px;
}

.rules-note {
  color: #64748b;
  font-size: 13px;
  margin-top: 8px;
}

.rule-preview {
  background-color: #f1f5f9;
  padding: 12px;
  border-radius: 6px;
  margin-bottom: 8px;
}

.rule-preview h5 {
  margin: 0 0 8px 0;
  font-size: 13px;
  color: #64748b;
  font-weight: 500;
}

.rules-json-preview {
  max-height: 300px;
}

.rules-file-info {
  margin-top: 8px;
  font-size: 13px;
  color: #64748b;
  display: flex;
  justify-content: flex-end;
}

.rules-actions {
  display: flex;
  gap: 10px;
}

.preview-btn {
  background-color: #94a3b8;
  color: white;
  flex: 1;
}

.preview-btn:hover {
  background-color: #64748b;
}

.send-rules-btn {
  background-color: #10b981;
  color: white;
  flex: 1;
}

.send-rules-btn:hover:not(:disabled) {
  background-color: #059669;
}

.rules-sent-info {
  display: flex;
  align-items: center;
  gap: 8px;
  background-color: #d1fae5;
  padding: 12px;
  border-radius: 6px;
  margin-top: 16px;
}

.success-icon {
  color: #059669;
  font-weight: bold;
  font-size: 18px;
}

.timestamp {
  margin-left: auto;
  font-size: 13px;
  color: #64748b;
}
</style>
