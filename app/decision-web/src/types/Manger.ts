import { ref, watch } from 'vue';
import { Node } from '@/types/Node';
import { Edge } from '@/types/Edge';
import { Point } from '@/types/Point';

export const activeToolId = ref<string>('cursor');


// ----------------------------节点设置----------------------------
export const nodes = ref<Node[]>([]);
export const edges = ref<Edge[]>([]);
export const points = ref<Point[]>([]);
// ----------------------------节点设置----------------------------

// ----------------------------地图设置----------------------------
export const mapWidth = ref(28);
export const mapHeight = ref(15);
export const MapSettingsPoints = ref<Point[]>([]);
// ----------------------------地图设置----------------------------


// 工具选择处理函数
export function handleToolSelected(toolId: string) {
    console.log('当前选中工具:', toolId);
};

watch(nodes, (newVal) => {
  if (newVal.length > 0 && newVal.length !== (nodes.value[nodes.value.length - 1].id.value + 1)) {
    nodes.value.forEach((node, index) => {
      if (node.id.value !== index) {
        const oldId = node.id.value;
        edges.value.filter(edge => edge.sourceId === oldId || edge.targetId === oldId).forEach(edge => {
          if (edge.sourceId === oldId) {
            edge.sourceId = index;
          } else if (edge.targetId === oldId) {
            edge.targetId = index;
          }
        });
        node.id.value = index;
      }
    });
  }
});

let oldVal: Edge[] = [];
watch(edges, (newVal) => {
  console.log("发生了变化",newVal.length, oldVal.length);

  // 找出哪些边被移除了
  const removedEdges = oldVal.filter(oldEdge =>
    !newVal.some(newEdge => newEdge.id.value === oldEdge.id.value)
  );

  // 找出哪些边是新增的
  const addedEdges = newVal.filter(newEdge =>
    !oldVal.some(oldEdge => oldEdge.id.value === newEdge.id.value)
  );

  // 处理移除的边
  if (removedEdges.length > 0) {
    // 避免直接修改节点的边
    nodes.value.forEach(node => {
      const filteredEdges = node.edges.filter(edge =>
        !removedEdges.some(e => e.id.value === edge.id.value)
      );
      if (filteredEdges.length !== node.edges.length) {
        node.edges = filteredEdges;
      }
    });
  }

  // 处理新增的边
  if (addedEdges.length > 0) {
    addedEdges.forEach(edge => {
      const sourceNode = nodes.value.find(node => node.id.value === edge.sourceId);
      if (sourceNode && !sourceNode.edges.some(e => e.id.value === edge.id.value)) {
        // 创建一个新数组而不是直接修改
        sourceNode.edges = [...sourceNode.edges, edge];
      }
    });
    console.log("新增的边", addedEdges);
  }

  // * 边的ID与索引不一致，使用延时更新，避免在当前监听循环中修改
  if (newVal.length > 0 && newVal.length !== (edges.value[edges.value.length - 1].id.value + 1)) {
    setTimeout(() => {
      const edgesToUpdate = [...edges.value];
      edgesToUpdate.forEach((edge, index) => {
        if (edge.id.value !== index) {
          edge.id.value = index;
        }
      });
      // 使用全新的数组引用触发响应式更新
      edges.value = edgesToUpdate;
    }, 0);
  }
  oldVal = [...newVal];
}, { deep: true }); // 添加深度监听选项

watch(points, (newVal) => {
  if (newVal.length > 0 && newVal.length !== (points.value[points.value.length - 1].id.value + 1)) {
    points.value.forEach((point, index) => {
      if (point.id.value !== index) {
        point.id.value = index;
      }
    });
  }
});

document.addEventListener('contextmenu', function(event) {
  event.preventDefault(); // 阻止默认行为,鼠标右键被屏蔽
});
