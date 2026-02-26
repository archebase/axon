<template>
  <div class="state-machine-diagram">
    <VueFlow
      v-model:nodes="nodes"
      v-model:edges="edges"
      :default-viewport="{ zoom: 1, x: 0, y: 0 }"
      :min-zoom="0.5"
      :max-zoom="2"
      fit-view-on-init
      :nodes-draggable="false"
      :nodes-connectable="false"
      :elements-selectable="false"
      :zoom-on-scroll="false"
      :pan-on-scroll="false"
      @ready="onFlowReady"
    >
    </VueFlow>
  </div>
</template>

<script setup>
import { ref, watch, onMounted, nextTick } from 'vue'
import { VueFlow } from '@vue-flow/core'
import '@vue-flow/core/dist/style.css'
import '@vue-flow/core/dist/theme-default.css'

const props = defineProps({
  currentState: {
    type: String,
    default: 'idle'
  }
})

// Force edge labels to be black using JavaScript
const forceEdgeLabelColor = () => {
  const setTextBlack = () => {
    // Select all text elements in Vue Flow
    const textElements = document.querySelectorAll(
      '.vue-flow__edgeLabel text, ' +
      '.vue-flow__edgeLabel tspan, ' +
      '.vue-flow__edge-textwrapper text, ' +
      '.vue-flow__edge-textwrapper tspan'
    )

    textElements.forEach(el => {
      el.setAttribute('fill', '#000000')
      el.setAttribute('color', '#000000')
      el.style.fill = '#000000'
      el.style.color = '#000000'
    })
  }

  // Execute immediately and also after a short delay to catch dynamically added elements
  setTextBlack()
  setTimeout(setTextBlack, 100)
  setTimeout(setTextBlack, 500)

  // Use MutationObserver to catch dynamically added elements
  const observer = new MutationObserver((mutations) => {
    setTextBlack()
  })

  const flowElement = document.querySelector('.vue-flow')
  if (flowElement) {
    observer.observe(flowElement, {
      childList: true,
      subtree: true,
      attributes: true,
      attributeFilter: ['class']
    })
  }

  return observer
}

const onFlowReady = () => {
  forceEdgeLabelColor()
}

onMounted(() => {
  // Also force color on mount
  nextTick(() => {
    forceEdgeLabelColor()
  })
})

// Define state machine nodes
const nodes = ref([
  {
    id: 'idle',
    type: 'default',
    position: { x: 0, y: 0 },
    data: {
      label: '💤\nIDLE',
      class: 'state-node idle-node'
    },
    class: 'custom-node'
  },
  {
    id: 'ready',
    type: 'default',
    position: { x: 300, y: 0 },
    data: {
      label: '✅\nREADY',
      class: 'state-node ready-node'
    },
    class: 'custom-node'
  },
  {
    id: 'recording',
    type: 'default',
    position: { x: 0, y: 150 },
    data: {
      label: '🔴\nRECORDING',
      class: 'state-node recording-node'
    },
    class: 'custom-node'
  },
  {
    id: 'paused',
    type: 'default',
    position: { x: 300, y: 150 },
    data: {
      label: '⏸️\nPAUSED',
      class: 'state-node paused-node'
    },
    class: 'custom-node'
  }
])

// All possible edges
const allEdges = [
  // Forward transitions
  {
    id: 'e-idle-ready',
    source: 'idle',
    target: 'ready',
    label: 'config',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'smoothstep',
    class: 'forward-edge'
  },
  {
    id: 'e-ready-recording',
    source: 'ready',
    target: 'recording',
    label: 'begin',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'smoothstep',
    class: 'forward-edge'
  },
  {
    id: 'e-recording-paused',
    source: 'recording',
    target: 'paused',
    label: 'pause',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'smoothstep',
    class: 'bidirectional-edge'
  },
  {
    id: 'e-paused-recording',
    source: 'paused',
    target: 'recording',
    label: 'resume',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'smoothstep',
    class: 'bidirectional-edge'
  },
  // Return transitions
  {
    id: 'e-ready-idle',
    source: 'ready',
    target: 'idle',
    label: 'clear',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'smoothstep',
    class: 'return-edge',
    style: { stroke: '#9ca3af', strokeWidth: 2 }
  },
  {
    id: 'e-recording-idle',
    source: 'recording',
    target: 'idle',
    label: 'finish',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'straight',
    class: 'return-edge',
    style: { stroke: '#9ca3af', strokeWidth: 2 }
  },
  {
    id: 'e-paused-idle',
    source: 'paused',
    target: 'idle',
    label: 'cancel',
    labelStyle: {
      fontSize: '11px',
      fontWeight: 600,
      fill: '#000000',
      color: '#000000',
      textFill: '#000000'
    },
    labelBgStyle: {
      fill: 'transparent',
      fillOpacity: 0
    },
    animated: false,
    type: 'smoothstep',
    class: 'return-edge cancel-edge',
    style: { stroke: '#dc2626', strokeWidth: 2 }
  }
]

// Define state machine edges (transitions)
const edges = ref([])

// Define which edges are visible for each state
const getVisibleEdges = (state) => {
  const edgeMap = {
    idle: ['e-idle-ready'],
    ready: ['e-ready-recording', 'e-ready-idle'],
    recording: ['e-recording-paused', 'e-recording-idle'],
    paused: ['e-paused-recording', 'e-paused-idle']
  }
  return edgeMap[state] || []
}

// Update node styles and visible edges based on current state
watch(() => props.currentState, (newState) => {
  nodes.value = nodes.value.map(node => {
    const isActive = node.id === newState
    return {
      ...node,
      data: {
        ...node.data,
        class: `state-node ${node.id}-node ${isActive ? 'active' : ''}`
      },
      style: isActive ? {
        borderColor: getNodeColor(node.id),
        borderWidth: '3px',
        boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
      } : {}
    }
  })

  // Update visible edges based on current state
  const visibleEdgeIds = getVisibleEdges(newState)
  edges.value = allEdges.filter(edge => visibleEdgeIds.includes(edge.id))

  // Force edge label colors after edges update
  nextTick(() => {
    forceEdgeLabelColor()
  })
}, { immediate: true })

function getNodeColor(stateId) {
  const colors = {
    idle: '#9ca3af',
    ready: '#10b981',
    recording: '#ef4444',
    paused: '#f59e0b'
  }
  return colors[stateId] || '#9ca3af'
}
</script>

<style scoped>
.state-machine-diagram {
  width: 100%;
  height: 350px;
  border-radius: 8px;
  overflow: hidden;
}

:deep(.vue-flow) {
  background: #fafafa;
}

:deep(.vue-flow__node) {
  padding: 0.75rem 1rem;
  border-radius: 8px;
  min-width: 75px;
  text-align: center;
  background: white;
  border: 2px solid #e5e7eb;
  box-shadow: 0 1px 2px rgba(0,0,0,0.05);
  transition: all 0.3s ease;
}

:deep(.vue-flow__node.custom-node .vue-flow__node-label) {
  font-size: 0.7rem;
  font-weight: 700;
  text-transform: uppercase;
  color: #374151;
  white-space: pre-line;
  line-height: 1.4;
}

:deep(.vue-flow__edge.forward-edge) {
  stroke: #9ca3af;
  stroke-width: 2;
  stroke-dasharray: none;
}

:deep(.vue-flow__edge.bidirectional-edge) {
  stroke: #9ca3af;
  stroke-width: 2;
  stroke-dasharray: none;
}

:deep(.vue-flow__edge.return-edge) {
  stroke: #9ca3af;
  stroke-width: 2;
  stroke-dasharray: 5;
}

:deep(.vue-flow__edge.cancel-edge) {
  stroke: #dc2626;
  stroke-width: 2;
  stroke-dasharray: 5;
}

:deep(.vue-flow__edgeLabel) {
  font-size: 0.7rem;
  font-weight: 700;
  text-transform: uppercase;
}

:deep(.vue-flow__edgeLabel text),
:deep(.vue-flow__edgeLabel tspan),
:deep(.vue-flow__edgeLabel *) {
  fill: #000000 !important;
  color: #000000 !important;
}

:deep(.vue-flow__edge-textwrapper text),
:deep(.vue-flow__edge-textwrapper tspan),
:deep(.vue-flow__edge-textwrapper *) {
  fill: #000000 !important;
  color: #000000 !important;
}

:deep(.vue-flow__edgeLabelbg),
:deep(.vue-flow__edge-textbg) {
  display: none !important;
}

:deep(.vue-flow__edge.forward-edge .vue-flow__edgeLabelbg),
:deep(.vue-flow__edge.forward-edge .vue-flow__edge-textbg) {
  display: none !important;
}

:deep(.vue-flow__edge.bidirectional-edge .vue-flow__edgeLabelbg),
:deep(.vue-flow__edge.bidirectional-edge .vue-flow__edge-textbg) {
  display: none !important;
}

:deep(.vue-flow__edge.return-edge .vue-flow__edgeLabelbg),
:deep(.vue-flow__edge.return-edge .vue-flow__edge-textbg) {
  display: none !important;
}

:deep(.vue-flow__edge.cancel-edge .vue-flow__edgeLabelbg),
:deep(.vue-flow__edge.cancel-edge .vue-flow__edge-textbg) {
  display: none !important;
}

:deep(.vue-flow__edge.forward-edge .vue-flow__edge-textwrapper text),
:deep(.vue-flow__edge.forward-edge .vue-flow__edge-textwrapper tspan),
:deep(.vue-flow__edge.forward-edge .vue-flow__edge-textwrapper *) {
  fill: #000000 !important;
  color: #000000 !important;
}

:deep(.vue-flow__edge.bidirectional-edge .vue-flow__edge-textwrapper text),
:deep(.vue-flow__edge.bidirectional-edge .vue-flow__edge-textwrapper tspan),
:deep(.vue-flow__edge.bidirectional-edge .vue-flow__edge-textwrapper *) {
  fill: #000000 !important;
  color: #000000 !important;
}

:deep(.vue-flow__edge.return-edge .vue-flow__edge-textwrapper text),
:deep(.vue-flow__edge.return-edge .vue-flow__edge-textwrapper tspan),
:deep(.vue-flow__edge.return-edge .vue-flow__edge-textwrapper *) {
  fill: #000000 !important;
  color: #000000 !important;
}

:deep(.vue-flow__edge.cancel-edge .vue-flow__edge-textwrapper text),
:deep(.vue-flow__edge.cancel-edge .vue-flow__edge-textwrapper tspan),
:deep(.vue-flow__edge.cancel-edge .vue-flow__edge-textwrapper *) {
  fill: #000000 !important;
  color: #000000 !important;
}

:deep(.vue-flow__controls) {
  display: none;
}

:deep(.idle-node.active) {
  background: linear-gradient(135deg, #f3f4f6, #e5e7eb);
  border-color: #9ca3af;
}

:deep(.ready-node.active) {
  background: linear-gradient(135deg, #d1fae5, #a7f3d0);
  border-color: #10b981;
}

:deep(.recording-node.active) {
  background: linear-gradient(135deg, #fee2e2, #fecaca);
  border-color: #ef4444;
}

:deep(.paused-node.active) {
  background: linear-gradient(135deg, #fef3c7, #fde68a);
  border-color: #f59e0b;
}
</style>
