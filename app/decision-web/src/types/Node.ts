import NodeBase from './NodeBase';
import { extendNodeWithDrag } from './extensions/node/nodeDrag';

// 扩展 Node 类
extendNodeWithDrag();

// 为了更好的类型支持，我们结合静态和实例类型
interface NodeStatic {
  draggingNodeId: { value: number | null };
  startDrag(event: MouseEvent, node: NodeBase, nodeList: NodeBase[]): void;
}

// 扩展Node的构造函数类型，包含静态成员
type NodeConstructor = typeof NodeBase & NodeStatic;

// 导出扩展后的Node
const Node = NodeBase as NodeConstructor;
export { Node };

// 再导出Node类型
export type Node = NodeBase;
