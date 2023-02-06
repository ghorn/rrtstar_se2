import * as THREE from "three";

import Stats from "three/addons/libs/stats.module.js";
import { GPUStatsPanel } from "three/addons/utils/GPUStatsPanel.js";
import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import wasmModule from "bazel-bin/js_vis/shim_wasm/shim.js";
import { LinesBuffer } from "./lines_buffer.js";

let renderer, scene, camera, controls;
let stats, gpuPanel;
let gui;
let lines_buffer = new LinesBuffer();
let goal_lines_buffer = new LinesBuffer();
let parent_node;

// load webassembly module
console.log("Initializing wasmModule...");
let cxx_shim_module = await wasmModule({
  onRuntimeInitialized() {
    console.log("Runtime initialized!");
  },
});

// create pathfinding problem
let r3_problem = new cxx_shim_module.R3Problem.SomeProblem();

// initialize and run the animation loop
init();
animate();

function init() {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x000000, 0.0);
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

  scene = new THREE.Scene();

  camera = new THREE.PerspectiveCamera(
    40,
    window.innerWidth / window.innerHeight,
    1,
    1000
  );
  camera.position.set(-6, 0, 8);

  controls = new OrbitControls(camera, renderer.domElement);
  controls.minDistance = 0.01;
  controls.maxDistance = 50;

  parent_node = new THREE.Object3D();
  parent_node.add(lines_buffer.get_segments());
  parent_node.add(goal_lines_buffer.get_segments());
  scene.add(parent_node);

  //

  window.addEventListener("resize", onWindowResize);
  onWindowResize();

  stats = new Stats();
  document.body.appendChild(stats.dom);

  gpuPanel = new GPUStatsPanel(renderer.getContext());
  stats.addPanel(gpuPanel);
  stats.showPanel(0);

  initGui();
}

function onWindowResize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();

  renderer.setSize(window.innerWidth, window.innerHeight);
}

function render() {
  const time = Date.now() * 0.001;

  parent_node.rotation.z = time * 0.5;

  // Iterate a few steps
  let target_num_edges = Math.min(5000, r3_problem.NumEdges() + 100);
  while (r3_problem.NumEdges() < target_num_edges) {
    r3_problem.Step();
  }

  // update opengl lines
  const bridge_lines = r3_problem.GetBridgeLines();
  lines_buffer.set_lines(bridge_lines);
  bridge_lines.delete();
  const goal_lines = r3_problem.GetGoalLine();
  goal_lines_buffer.set_lines(goal_lines);
  goal_lines.delete();

  // main scene
  renderer.setClearColor(0x000000, 0);

  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);

  // renderer will set this eventually
  // goal_lines_buffer.matLine.resolution.set(
  //   window.innerWidth,
  //   window.innerHeight
  // ); // resolution of the viewport
  // goal_lines_buffer.matLine.needsUpdate = true;

  gpuPanel.startQuery();
  renderer.render(scene, camera);
  gpuPanel.endQuery();
}

function animate() {
  requestAnimationFrame(animate);
  stats.update();

  render(scene, camera);
}

//

function initGui() {
  gui = new GUI();

  const param = {
    "line type": 0,
    "world units": false,
    num_points: 500,
    width: 5,
    alphaToCoverage: true,
    dashed: false,
    "dash scale": 1,
    "dash / gap": 1,
  };

  // gui
  //   .add(param, "line type", { LineGeometry: 0, "gl.LINE": 1 })
  //   .onChange(function (val) {
  //     switch (val) {
  //       case 0:
  //         line.visible = true;
  //         lineGl.visible = false;
  //         break;

  //       case 1:
  //         line.visible = false;
  //         lineGl.visible = true;
  //         break;
  //     }
  //   });

  // gui.add(param, "world units").onChange(function (val) {
  //   matLine.worldUnits = val;
  //   matLine.needsUpdate = true;
  // });

  gui.add(param, "num_points", 1, 1000).onChange(function (val) {
    num_points = Math.round(val);
  });

  // gui.add(param, "alphaToCoverage").onChange(function (val) {
  //   matLine.alphaToCoverage = val;
  // });

  // gui.add(param, "dashed").onChange(function (val) {
  //   matLine.dashed = val;
  //   lineGl.material = val ? matLineDashed : matLineBasic;
  // });

  // gui.add(param, "dash scale", 0.5, 2, 0.1).onChange(function (val) {
  //   matLine.dashScale = val;
  //   matLineDashed.scale = val;
  // });

  // gui
  //   .add(param, "dash / gap", { "2 : 1": 0, "1 : 1": 1, "1 : 2": 2 })
  //   .onChange(function (val) {
  //     switch (val) {
  //       case 0:
  //         matLine.dashSize = 2;
  //         matLine.gapSize = 1;

  //         matLineDashed.dashSize = 2;
  //         matLineDashed.gapSize = 1;

  //         break;

  //       case 1:
  //         matLine.dashSize = 1;
  //         matLine.gapSize = 1;

  //         matLineDashed.dashSize = 1;
  //         matLineDashed.gapSize = 1;

  //         break;

  //       case 2:
  //         matLine.dashSize = 1;
  //         matLine.gapSize = 2;

  //         matLineDashed.dashSize = 1;
  //         matLineDashed.gapSize = 2;

  //         break;
  //     }
  //   });
}
