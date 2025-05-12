import { ref, watch } from 'vue';
import { Node , type NodeGroup} from '@/types/Node';
import { Edge } from '@/types/Edge';
import { Point } from '@/types/Point';
import { type Area } from '@/types/Area';

export const activeToolId = ref<string>('cursor');


// ----------------------------对象管理----------------------------
export const nodes = ref<Node[]>([]);
export const edges = ref<Edge[]>([]);
export const points = ref<Point[]>([]);
export const areas = ref<Area[]>([]);
export const NodeGroups = ref<NodeGroup[]>([]);
// ----------------------------对象管理----------------------------


// ----------------------------地图设置----------------------------
export const mapWidth = ref(28);
export const mapHeight = ref(15);
export const MapSettingsPoints = ref<Point[]>([]);
// ----------------------------地图设置----------------------------

// ----------------------------机器人位置----------------------------
export const RobotPose = ref<Point>();
// ----------------------------机器人位置----------------------------

export const mousePositionCanvas = ref<{ x: number, y: number }>({ x: 0, y: 0 });
export const currentScale = ref(1.0);
// 工具选择处理函数
export function handleToolSelected(toolId: string) {
    console.log('当前选中工具:', toolId);
};

export function clearAll() {
  nodes.value.splice(0, nodes.value.length);
  edges.value.splice(0, edges.value.length);
  points.value.splice(0, points.value.length);
  areas.value.splice(0, areas.value.length);
  NodeGroups.value.splice(0, NodeGroups.value.length);
  oldNodes = [];
  oldVal = [];
}

let oldNodes: Node[] = [];
watch(nodes, (newVal) => {
  if (oldNodes.length > newVal.length) {
    const removedNodes = oldNodes.filter(oldNode => !newVal.some(newNode => newNode.id.value === oldNode.id.value));
    removedNodes.forEach(node => {
      // 移除节点后，移除与该节点相连的边
      edges.value = [...edges.value.filter(edge => edge.sourceId !== node.id.value && edge.targetId !== node.id.value)];
      // 移除节点后，移除节点组中节点ID
      NodeGroups.value.forEach(group => {
        group.nodesId = group.nodesId.filter(id => id !== node.id.value);
      });
    });
  }
  oldNodes = [...newVal];
}, { deep: true });

let oldVal: Edge[] = [];
watch(edges, (newVal) => {
  console.log("edges发生了变化",newVal.length, oldVal.length);

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
      const filteredEdges = node.edges.filter(edgeId =>
        !removedEdges.some(e => e.id.value === edgeId.value)
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
      if (sourceNode && sourceNode.id.value === edge.sourceId) {
        sourceNode.updataEdges(edge.id.value);
      }
    });
    console.log("新增的边", addedEdges);
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

export function showError(message: string) {
  window.dispatchEvent(new CustomEvent('custom-error', {
    detail: { message: message }
  }));
}


export function GetNewId(ids: number[]): number {
  if (ids.length === 0) return 0;
  ids.sort((a, b) => a - b);

  let expected = 0;
  for (const id of ids) {
      if (id > expected) {
          return expected;
      }
      expected = id + 1;
  }
  return expected;
}