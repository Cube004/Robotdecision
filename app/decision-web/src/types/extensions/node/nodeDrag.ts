import { ref } from 'vue';
import type { Ref } from 'vue';
import Node from '@/types/NodeBase';
import { mousePositionCanvas } from '@/types/Manger';
// 为静态方法定义类型接口
interface NodeConstructor {
  new (
    id: { value: number },
    position: { x: number, y: number },
    shape: { width: number, height: number, borderWidth: number, borderRadius: number },
    color: { borderColor: string, fillColor: string, fillOpacity: number },
    text?: { size: number, color: string, content: string, fontFamily: string } | null,
    taskConfig?: { nodeType: string, mode: string, waypoint: unknown | null, resetTime: number | null } | null,
    icon?: { text?: string, bgColor?: string, src?: string, svgPath?: string } | null
  ): Node;
  startDrag(event: MouseEvent, node: Node, nodeList: Node[]): void;
  draggingNodeId: Ref<number | null>;
}

// 扩展 Node 类添加静态属性和方法
export function extendNodeWithDrag() {
  // 添加静态属性
  (Node as unknown as NodeConstructor).draggingNodeId = ref<number | null>(null);

  // 添加静态方法
  (Node as unknown as NodeConstructor).startDrag = function(event: MouseEvent, node: Node, nodeList: Node[]) {
    event.preventDefault();

    // 记录当前拖拽的节点ID
    (Node as unknown as NodeConstructor).draggingNodeId.value = node.id.value;

    // 添加全局事件监听
    const handleDragBound = (e: MouseEvent) => handleDrag(e, nodeList);
    const stopDragBound = () => stopDrag(nodeList, handleDragBound, stopDragBound);

    window.addEventListener('mousemove', handleDragBound);
    window.addEventListener('mouseup', stopDragBound);
  };

  // 处理拖拽移动
  function handleDrag(event: MouseEvent, nodeList: Node[]) {
    if ((Node as unknown as NodeConstructor).draggingNodeId.value !== null) {
      const node = nodeList.find(node => node.id.value === (Node as unknown as NodeConstructor).draggingNodeId.value);
      if (node) {
        node.updatePosition(
          mousePositionCanvas.value.x - node.shape.width / 2,
          mousePositionCanvas.value.y - node.shape.height / 2
        )
      }
    }
  }

  // 停止拖拽
  function stopDrag(nodeList: Node[], handleDragBound: (e: MouseEvent) => void, stopDragBound: () => void) {
    (Node as unknown as NodeConstructor).draggingNodeId.value = null;

    // 移除全局事件监听
    window.removeEventListener('mousemove', handleDragBound);
    window.removeEventListener('mouseup', stopDragBound);
  }
}

// 为了保持兼容性，导出对静态属性的引用
export const draggingNodeId = () => (Node as unknown as NodeConstructor).draggingNodeId;
