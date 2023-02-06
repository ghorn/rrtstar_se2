import * as THREE from "three";

import Stats from "three/addons/libs/stats.module.js";
import { GPUStatsPanel } from "three/addons/utils/GPUStatsPanel.js";

import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import wasmModule from "bazel-bin/js_vis/shim_wasm/shim.js";

class LinesBuffer {
  MAX_POINTS = 20000;

  constructor() {
    const positions = new Float32Array(this.MAX_POINTS * 3); // 3 vertices per point
    const colors = new Float32Array(this.MAX_POINTS * 4); // RGB for each point

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute(
      "position",
      new THREE.BufferAttribute(positions, 3).setUsage(THREE.DynamicDrawUsage) // TODO(greg): try StreamDrawUsage
    );
    geometry.setAttribute(
      "color",
      new THREE.BufferAttribute(colors, 4).setUsage(THREE.DynamicDrawUsage) // TODO(greg): try StreamDrawUsage
    );
    geometry.setDrawRange(0, 0); // initially draw nothing

    const material = new THREE.LineBasicMaterial({
      vertexColors: true,
      blending: THREE.AdditiveBlending,
      transparent: true,
    });

    this.lines = new THREE.LineSegments(geometry, material);
    // this.lines = new THREE.Line(geometry, material);
    this.lines.visible = true;
  }

  set_lines(line_list) {
    const positions = this.lines.geometry.attributes.position.array;
    const colors = this.lines.geometry.attributes.color.array;

    this.lines.geometry.clearGroups();

    let segment_indices = [];
    // let segment_index = 0; // designate where each segment begins
    let count = 0; // one for each point in the geometry position buffer

    // console.log(line_list);
    // let goal_line_length = 0;
    for (let i = 0; i < line_list.size(); i++) {
      const line = line_list.get(i);
      // segment_index += line.size();
      this.lines.geometry.addGroup({ start: count, count: line.size() });

      // goal_line_length = line.size();
      // append segment index to the segment_indices array
      // segment_indices.push(segment_index, segment_index + 1);

      for (let k = 0; k < line.size(); k++) {
        // segment_indices.push(count, count + 1);
        segment_indices.push(count);
        const xyz_rgb = line.get(k);

        positions[3 * count] = xyz_rgb.x;
        positions[3 * count + 1] = xyz_rgb.y;
        positions[3 * count + 2] = xyz_rgb.z;
        colors[4 * count] = xyz_rgb.r;
        colors[4 * count + 1] = xyz_rgb.g;
        colors[4 * count + 2] = xyz_rgb.b;
        colors[4 * count + 3] = 1; //xyz_rgb.a;

        // console.log("x: " + xyz_rgb.x);
        // console.log("y: " + xyz_rgb.y);
        // console.log("z: " + xyz_rgb.z);
        // console.log("r: " + xyz_rgb.r);
        // console.log("g: " + xyz_rgb.g);
        // console.log("b: " + xyz_rgb.b);
        count++;
      }
      line.delete();
    }
    // console.log(segment_indices);
    this.lines.geometry.attributes.position.needsUpdate = true; // required after updates
    this.lines.geometry.attributes.color.needsUpdate = true; // required after updates

    // this.geometry.setFromPoints(pos_col);
    this.lines.geometry.computeBoundingBox();
    this.lines.geometry.computeBoundingSphere();
    this.lines.geometry.setDrawRange(0, count);
    // segment_indices = [0, 1, 1, 2, 2, 3];
    // this.lines.geometry.setDrawRange(0, 6);
    // this.lines.geometry.setIndex(segment_indices);
    // console.log("goal line length: " + goal_line_length);
  }

  get_segments() {
    return this.lines;
  }
}

let renderer, scene, camera, controls;
let stats, gpuPanel;
let gui;
let lines_buffer = new LinesBuffer();
let goal_lines_buffer = new LinesBuffer();
// let num_points = 600;

console.log("Initializing wasmModule...");
let cxx_shim_module = await wasmModule({
  onRuntimeInitialized() {
    console.log("Runtime initialized!");
  },
});
let r3_problem = new cxx_shim_module.R3Problem.SomeProblem();
console.log("Created r3 problem");
console.log(r3_problem);
//   const instance = await instance_promise;
//   console.log("calling C++ function...");
//   console.log(instance.sayHello2());
//   console.log("Hello was said!");
// } catch (e) {
//   console.log("Got an error");
//   console.log(e);
// }

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

  scene.add(lines_buffer.get_segments());
  scene.add(goal_lines_buffer.get_segments());

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

function animate() {
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

  requestAnimationFrame(animate);

  stats.update();

  // main scene
  renderer.setClearColor(0x000000, 0);

  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);

  // renderer will set this eventually
  // matLine.resolution.set(window.innerWidth, window.innerHeight); // resolution of the viewport

  gpuPanel.startQuery();
  renderer.render(scene, camera);
  gpuPanel.endQuery();
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
