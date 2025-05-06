<template>
  <div>
    <canvas ref="backgroundLayer"></canvas>
    <EdgeComponent
      :arrow-offset="0"
      :alignment-tolerance="15"
      @edge-click="handleEdgeClick"
    />
    <NodeItem @open-edit-menu="handleNodeClick" />

    <!-- 工具菜单 -->
    <ToolMenu />

    <!-- 编辑菜单 -->
    <EditMenu
      :visible="editMenuVisible"
      :selected-node-id="selectedNodeId"
      @update:visible="editMenuVisible = $event"
      @node-updated="handleNodeUpdated"
    />
    <EdgeMenu
      :visible="edgeMenuVisible"
      :selected-edge-id="selectedEdgeId"
      @update:visible="edgeMenuVisible = $event"
      @edge-updated="handleEdgeUpdated"
      @close="edgeMenuVisible = false"
    />
  </div>
</template>

<script setup lang="ts" name="App">
  import NodeItem from '@/components/NodeItem.vue';
  import EdgeComponent from '@/components/EdgeComponent.vue';
  import ToolMenu from '@/components/ToolMenu.vue';
  import EditMenu from '@/components/NodeMenu.vue';
  import { onMounted, ref } from 'vue';
  import { example } from '@/types/example';
  import EdgeMenu from '@/components/EdgeMenu.vue';
  import type { Edge } from '@/types/EdgeBase';

  // 定义节点和边的响应式数据
  const backgroundLayer = ref<HTMLCanvasElement | null>(null);
  const editMenuVisible = ref(false);
  const selectedNodeId = ref<number | null>(null);

  // EdgeMenu相关状态
  const edgeMenuVisible = ref(false);
  const selectedEdgeId = ref<number | null>(null);

  // 处理节点点击事件
  const handleNodeClick = (nodeId: number) => {
    selectedNodeId.value = nodeId;
    editMenuVisible.value = true;

    // 关闭边编辑菜单
    edgeMenuVisible.value = false;
  };

  // 处理边点击事件
  const handleEdgeClick = (edgeId: number) => {
    selectedEdgeId.value = edgeId;
    edgeMenuVisible.value = true;

    // 关闭节点编辑菜单
    editMenuVisible.value = false;
  };

  // 处理节点更新事件
  const handleNodeUpdated = (node: Node & { type?: string; mode?: string; resetTime?: number; waypointId?: string }) => {
    console.log('节点已更新', node);
    // 这里可以添加其他逻辑，例如保存到服务器
  };

  // 处理边更新事件
  const handleEdgeUpdated = (edge: Edge) => {
    console.log('连线已更新', edge);
    // 这里可以添加其他逻辑，例如保存到服务器
  };

  onMounted(() => {
    // 绘制背景网格
    const ctx = backgroundLayer.value?.getContext('2d');
    if (backgroundLayer.value && ctx){
      backgroundLayer.value.width = window.innerWidth;
      backgroundLayer.value.height = window.innerHeight;

      ctx.fillStyle = '#F2F4F7';
      ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);
      const gridInterval = window.innerWidth / 100;
      // 计算最大有效索引
      const maxXIndex = Math.floor(backgroundLayer.value.width / gridInterval);
      const maxYIndex = Math.floor(backgroundLayer.value.height / gridInterval);

      for (let xIndex = 0; xIndex <= maxXIndex; xIndex++) {
          for (let yIndex = 0; yIndex <= maxYIndex; yIndex++) {
              const x = xIndex * gridInterval;
              const y = yIndex * gridInterval;

              ctx.beginPath();
              ctx.fillStyle = '#E2E4EC';
              ctx.arc(x, y, 1.5 , 0, Math.PI * 2);
              ctx.fill();
          }
      }
    }
    example();
  })
</script>


<style scoped>
  canvas {
    position: absolute;
    top: 0;
    left: 0;
    width: 100%;
    height: 100%;
  }

  .control-panel {
    position: fixed;
    top: 20px;
    right: 20px;
    background-color: white;
    padding: 15px;
    border-radius: 8px;
    box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
    z-index: 100;
    min-width: 250px;
  }

  .control-item {
    margin-bottom: 10px;
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .control-item.checkbox {
    margin-top: 15px;
  }

  .control-item.checkbox label {
    display: flex;
    align-items: center;
    gap: 5px;
    cursor: pointer;
  }

  .control-item label {
    font-size: 14px;
    color: #334155;
    margin-right: 10px;
  }

  .control-item input[type="range"] {
    width: 120px;
    margin: 0 10px;
  }

  .control-item span {
    font-size: 14px;
    color: #64748B;
    min-width: 40px;
    text-align: right;
  }
</style>
