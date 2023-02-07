import * as THREE from "three";

import Stats from "three/addons/libs/stats.module.js";
import { GPUStatsPanel } from "three/addons/utils/GPUStatsPanel.js";
import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import wasmModule from "bazel-bin/js_vis/shim_wasm/shim.js";
import { LinesBuffer } from "./lines_buffer.js";

class R3ProblemScene {
  constructor() {
    this.lines_buffer = new LinesBuffer();
    this.goal_lines_buffer = new LinesBuffer();
    this.goal_region_sphere = new THREE.Mesh(
      new THREE.SphereGeometry(),
      // new THREE.MeshBasicMaterial({ color: 0x00ff00, transparent: true, opacity: 0.5 })
      new THREE.MeshBasicMaterial({
        color: 0xffff00,
        transparent: true,
        opacity: 0.5,
      })
    );

    this.obstacle_meshes = [];

    this.parent_node = new THREE.Object3D();
    this.parent_node.add(this.lines_buffer.get_segments());
    this.parent_node.add(this.goal_lines_buffer.get_segments());
    this.parent_node.add(this.goal_region_sphere);

    this.scene = new THREE.Scene();
    this.scene.add(this.parent_node);
  }

  update_obstacles(r3_problem, show_obstacles, obstacle_opacity) {
    const problem_obstacles = r3_problem.GetObstacles();
    const num_problem_obstacles = problem_obstacles.size();

    // if there are too few obstacles, add new ones
    if (num_problem_obstacles > this.obstacle_meshes.length) {
      for (
        let i = this.obstacle_meshes.length;
        i < num_problem_obstacles;
        i++
      ) {
        const new_obstacle_mesh = new THREE.Mesh(
          new THREE.SphereGeometry(),
          new THREE.MeshBasicMaterial({
            color: 0xff0000,
            transparent: true,
            opacity: 0.5,
          })
        );
        this.parent_node.add(new_obstacle_mesh);
        this.obstacle_meshes.push(new_obstacle_mesh);
      }
    }

    // if there are too many obstacles, remove extras from the scene and delete the meshes
    if (num_problem_obstacles < this.obstacle_meshes.length) {
      const extranious_obstacle_meshes = this.obstacle_meshes.slice(
        num_problem_obstacles - this.obstacle_meshes.length
      );
      for (let i = 0; i < extranious_obstacle_meshes.length; i++) {
        const obstacle_mesh = extranious_obstacle_meshes[i];
        obstacle_mesh.geometry.dispose();
        obstacle_mesh.material.dispose();
        this.parent_node.remove(obstacle_mesh);
      }

      this.obstacle_meshes = this.obstacle_meshes.slice(
        0,
        num_problem_obstacles
      );
    }

    // now update all the obstacles
    for (let i = 0; i < num_problem_obstacles; i++) {
      const obstacle = problem_obstacles.get(i);
      this.obstacle_meshes[i].scale.set(
        obstacle.radius,
        obstacle.radius,
        obstacle.radius
      );
      this.obstacle_meshes[i].position.set(
        obstacle.center.x,
        obstacle.center.y,
        obstacle.center.z
      );

      // visibility
      if (show_obstacles) {
        this.obstacle_meshes[i].visible = true;
      } else {
        this.obstacle_meshes[i].visible = false;
      }

      // opacity
      const material = this.obstacle_meshes[i].material;
      if (material.opacity != obstacle_opacity) {
        material.opacity = obstacle_opacity;
      }
    }
  }

  update(r3_problem, gui_params) {
    // set the goal region
    const problem_goal_region = r3_problem.GetGoalRegion();
    this.goal_region_sphere.scale.set(
      problem_goal_region.radius,
      problem_goal_region.radius,
      problem_goal_region.radius
    );
    this.goal_region_sphere.position.set(
      problem_goal_region.center.x,
      problem_goal_region.center.y,
      problem_goal_region.center.z
    );

    // update obstacles
    this.update_obstacles(
      r3_problem,
      gui_params.show_obstacles,
      gui_params.obstacle_opacity
    );

    // set the pathfinding lines
    const bridge_lines = r3_problem.GetBridgeLines();
    this.lines_buffer.set_lines(bridge_lines);
    bridge_lines.delete();

    // set the optimal path lines
    const goal_lines = r3_problem.GetGoalLine();
    this.goal_lines_buffer.set_lines(goal_lines);
    goal_lines.delete();
  }
}

let renderer, scene, camera, controls;
let stats, gpuPanel;
let gui;
let r3_problem_scene = new R3ProblemScene();

let last_rotation_time = Date.now();
let last_solve_time = null;

// load webassembly module
console.log("Initializing wasmModule...");
let cxx_shim_module = await wasmModule({
  onRuntimeInitialized() {
    console.log("Runtime initialized!");
  },
});

// create pathfinding problem
var problem_factory = new cxx_shim_module.R3ProblemFactory();

const gui_params = {
  rotate: true,
  rotation_rate: 0.5,
  max_iterations: 5000,
  iterations_per_frame: 200,
  delay_before_restart: 1,
  scene: {
    show_obstacles: true,
    obstacle_opacity: 0.2,
  },
  problem: {
    max_num_obstacles: 10,
    obstacle_fraction: 0.6,
    min_length: 3,
    max_length: 4,
  },
};

let r3_problem = problem_factory.RandomProblem(gui_params.problem);

// initialize and run the animation loop
init();
animate();

function init() {
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setClearColor(0x000000, 0.0);
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

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
  // optionally rotate
  const now = Date.now();
  const delta_time = now - last_rotation_time;
  last_rotation_time = now;
  if (gui_params.rotate) {
    r3_problem_scene.parent_node.rotation.y +=
      delta_time * 0.001 * gui_params.rotation_rate;
  }

  // Iterate a few steps
  let target_num_edges = Math.min(
    gui_params.max_iterations,
    r3_problem.NumEdges() + gui_params.iterations_per_frame
  );
  const was_solved = r3_problem.NumEdges() == gui_params.max_iterations;

  while (r3_problem.NumEdges() < target_num_edges) {
    r3_problem.Step();
  }
  const is_solved = r3_problem.NumEdges() == gui_params.max_iterations;

  // if problem was just solved, set the time
  if (!was_solved && is_solved) {
    last_solve_time = now;
  }

  // optionally reset problem
  if (
    is_solved &&
    now - last_solve_time > 1000 * gui_params.delay_before_restart
  ) {
    r3_problem.delete();
    r3_problem = problem_factory.RandomProblem();
  }

  // update opengl lines
  r3_problem_scene.update(r3_problem, gui_params.scene);

  // main scene
  renderer.setClearColor(0x000000, 0);

  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);

  gpuPanel.startQuery();
  renderer.render(r3_problem_scene.scene, camera);
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

  gui.add(gui_params, "rotate");
  gui.add(gui_params, "rotation_rate", 0, 1.5);
  gui.add(gui_params, "max_iterations", 0, 10000);
  gui.add(gui_params, "iterations_per_frame", 1, 1000, 20);
  gui.add(gui_params, "delay_before_restart", 0.1, 5, 0.1);

  const problem_folder = gui.addFolder("problem");
  problem_folder.add(gui_params.problem, "max_num_obstacles", 0, 20, 1);
  problem_folder.add(gui_params.problem, "obstacle_fraction", 0, 1, 0.01);
  const min_length = problem_folder.add(
    gui_params.problem,
    "min_length",
    0.05,
    4.95,
    0.05
  );
  const max_length = problem_folder.add(
    gui_params.problem,
    "max_length",
    0.1,
    5,
    0.05
  );
  // make sure max_length is bigger than min_length
  min_length.onChange(function (min_length_val) {
    const max_length_val = max_length.getValue();
    if (min_length_val > max_length_val) {
      max_length.setValue(min_length_val + 0.05);
      max_length.updateDisplay();
    }
  });
  // make sure min length is smaller than max length
  max_length.onChange(function (max_length_val) {
    const min_length_val = min_length.getValue();
    if (max_length_val < min_length_val) {
      min_length.setValue(max_length_val - 0.05);
      min_length.updateDisplay();
    }
  });

  const scene_folder = gui.addFolder("scene");
  scene_folder.add(gui_params.scene, "show_obstacles");
  scene_folder.add(gui_params.scene, "obstacle_opacity", 0, 1, 0.01);

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

  // gui.add(gui_params, "num_points", 1, 1000).onChange(function (val) {
  //   num_points = Math.round(val);
  // });

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
