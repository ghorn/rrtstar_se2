import * as THREE from "three";

import Stats from "three/addons/libs/stats.module.js";
import { GPUStatsPanel } from "three/addons/utils/GPUStatsPanel.js";
import { GUI } from "three/addons/libs/lil-gui.module.min.js";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import wasmModule from "bazel-bin/js_vis/shim_wasm/shim.js";
import { LinesBuffer } from "./lines_buffer.js";

class ProblemScene {
  constructor() {
    this.lines_buffer = new LinesBuffer();
    this.goal_lines_buffer = new LinesBuffer();
    this.bounding_box_lines_buffer = new LinesBuffer();
    this.axes_lines_buffer = new LinesBuffer();
    this.axes_lines_buffer.set_lines_from_lists([
      [
        { x: 0, y: 0, z: 0, r: 1, g: 0, b: 0, a: 1 },
        { x: 1, y: 0, z: 0, r: 1, g: 0, b: 0, a: 1 },
      ],
      [
        { x: 0, y: 0, z: 0, r: 0, g: 1, b: 0, a: 1 },
        { x: 0, y: 1, z: 0, r: 0, g: 1, b: 0, a: 1 },
      ],
      [
        { x: 0, y: 0, z: 0, r: 0, g: 0, b: 1, a: 1 },
        { x: 0, y: 0, z: 1, r: 0, g: 0, b: 1, a: 1 },
      ],
    ]);
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
    this.parent_node.rotateX(Math.PI / 2);
    this.parent_node.add(this.lines_buffer.get_segments());
    this.parent_node.add(this.goal_lines_buffer.get_segments());
    this.parent_node.add(this.bounding_box_lines_buffer.get_segments());
    this.parent_node.add(this.goal_region_sphere);
    this.parent_node.add(this.axes_lines_buffer.get_segments());

    this.scene = new THREE.Scene();
    this.scene.add(this.parent_node);
  }

  update_obstacles(problem_obstacles, show_obstacles, obstacle_opacity) {
    const num_problem_obstacles = problem_obstacles.length;

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
      const obstacle = problem_obstacles[i];
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

  update_goal_region(
    problem_goal_region,
    show_goal_region,
    goal_region_opacity
  ) {
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
    if (show_goal_region) {
      this.goal_region_sphere.visible = true;
    } else {
      this.goal_region_sphere.visible = false;
    }
    // update goal material
    const material = this.goal_region_sphere.material;
    if (material.opacity != goal_region_opacity) {
      material.opacity = goal_region_opacity;
    }
  }

  update_common(
    goal_region,
    obstacles,
    bridge_lines,
    goal_lines,
    bounding_box_lines,
    gui_params
  ) {
    // set the goal region
    this.update_goal_region(
      goal_region,
      gui_params.show_goal_region,
      gui_params.goal_region_opacity
    );

    // update obstacles
    this.update_obstacles(
      obstacles,
      gui_params.show_obstacles,
      gui_params.obstacle_opacity
    );

    // set the pathfinding lines
    this.lines_buffer.set_lines(bridge_lines);

    // set the optimal path lines
    this.goal_lines_buffer.set_lines(goal_lines);

    // set the bounding box lines
    this.bounding_box_lines_buffer.set_lines(bounding_box_lines);
    if (gui_params.show_bounding_box) {
      this.bounding_box_lines_buffer.lines.visible = true;
    } else {
      this.bounding_box_lines_buffer.lines.visible = false;
    }

    // update the axes
    if (gui_params.show_axes) {
      this.axes_lines_buffer.lines.visible = true;
    } else {
      this.axes_lines_buffer.lines.visible = false;
    }
  }

  update_r3(r3_problem, gui_params) {
    // get the goal region
    const goal_region = r3_problem.GetGoalRegion();

    // get obstacles and convert to a list
    const obstacles_vec = r3_problem.GetObstacles();
    const obstacles = [];
    for (let i = 0; i < obstacles_vec.size(); i++) {
      obstacles.push(obstacles_vec.get(i));
    }
    obstacles_vec.delete();

    // get the lines
    const bridge_lines = r3_problem.GetBridgeLines();
    const goal_lines = r3_problem.GetGoalLine();
    const bounding_box_lines = r3_problem.GetBoundingBoxLines(
      gui_params.bounding_box_opacity
    );

    // update the scene
    this.update_common(
      goal_region,
      obstacles,
      bridge_lines,
      goal_lines,
      bounding_box_lines,
      gui_params
    );

    // delete C++ objects
    goal_lines.delete();
    bridge_lines.delete();
    bounding_box_lines.delete();
  }

  update_se2(se2_problem, gui_params) {
    // Get the goal region.
    // It's in the form of {position: {center: {x, y}, radius}, min_angle, max_angle}.
    // Convert to {center: {x, y, z}, radius}.
    const goal_region = se2_problem.GetGoalRegion().position;
    goal_region.center.z = 0;

    // get obstacles and set z to 0
    const obstacles_vec = se2_problem.GetObstacles();
    const obstacles = [];
    for (let i = 0; i < obstacles_vec.size(); i++) {
      const obstacle = obstacles_vec.get(i);
      obstacle.center.z = 0;
      obstacles.push(obstacle);
    }
    obstacles_vec.delete();

    // get the lines
    const bridge_lines = se2_problem.GetBridgeLines();
    const goal_lines = se2_problem.GetGoalLine();
    const bounding_box_lines = se2_problem.GetBoundingBoxLines(
      gui_params.bounding_box_opacity
    );

    // update the scene
    this.update_common(
      goal_region,
      obstacles,
      bridge_lines,
      goal_lines,
      bounding_box_lines,
      gui_params
    );

    // delete C++ objects
    bridge_lines.delete();
    goal_lines.delete();
    bounding_box_lines.delete();
  }
}

let renderer, scene, camera, controls;
let stats, gpuPanel;
let gui;
let problem_scene = new ProblemScene();

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
var problem_factory = new cxx_shim_module.ProblemFactory();

const gui_params = {
  delay_before_restart: 1,
  space: "r3",
  scene: {
    rotate: true,
    rotation_rate: 0.5,
    show_goal_region: true,
    goal_region_opacity: 0.5,
    show_obstacles: true,
    obstacle_opacity: 0.2,
    show_bounding_box: true,
    bounding_box_opacity: 0.75,
    show_axes: false,
  },
  r3_problem: {
    max_iterations: 5000,
    iterations_per_frame: 200,
    eta: 0.55,
    max_num_obstacles: 10,
    obstacle_fraction: 0.6,
    min_length: 3,
    max_length: 4,
    goal_radius: 0.5,
  },
  se2_problem: {
    max_iterations: 2000,
    iterations_per_frame: 20,
    rho: 0.6, // radius of curvature
    eta: 4.5,
    max_num_obstacles: 20,
    obstacle_fraction: 0.5,
    min_length: 3,
    max_length: 5,
    goal_radius: 0.5,
  },
};

let problem;
if (gui_params.space == "r3") {
  problem = problem_factory.RandomR3Problem(gui_params.r3_problem);
} else if (gui_params.space == "se2") {
  problem = problem_factory.RandomSe2Problem(
    gui_params.se2_problem,
    gui_params.se2_problem.rho
  );
} else {
  throw "Unknown space: " + gui_params.space;
}

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
  camera.position.set(-4, 4, 5);

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
  if (gui_params.scene.rotate) {
    problem_scene.parent_node.rotation.z +=
      delta_time * 0.001 * gui_params.scene.rotation_rate;
  }

  // get the appropriate problem params
  let problem_params;
  if (problem.constructor.name == "R3Problem") {
    problem_params = gui_params.r3_problem;
  } else if (problem.constructor.name == "Se2Problem") {
    problem_params = gui_params.se2_problem;
  } else {
    throw "Unknown class: " + problem.constructor.name;
  }

  // Iterate a few steps
  let target_num_edges = Math.min(
    problem_params.max_iterations,
    problem.NumEdges() + problem_params.iterations_per_frame
  );
  const was_finished = problem.NumEdges() >= problem_params.max_iterations;

  let num_failed_iterations = 0;
  while (problem.NumEdges() < target_num_edges) {
    if (!problem.Step()) {
      num_failed_iterations++;
      if (num_failed_iterations > 5000) {
        console.warn("Too many failed iterations. Giving up.");
        break;
      }
    }
  }
  const is_finished = problem.NumEdges() >= problem_params.max_iterations;

  // if problem was just finished, set the time
  if (!was_finished && is_finished) {
    last_solve_time = now;
  }

  // optionally reset problem
  if (
    is_finished &&
    now - last_solve_time > 1000 * gui_params.delay_before_restart
  ) {
    problem.delete();

    // create a new problem in the proper space
    if (gui_params.space == "r3") {
      problem = problem_factory.RandomR3Problem(gui_params.r3_problem);
    } else if (gui_params.space == "se2") {
      problem = problem_factory.RandomSe2Problem(
        gui_params.se2_problem,
        gui_params.se2_problem.rho
      );
    } else {
      throw "Unknown space: " + gui_params.space;
    }
  }

  // update opengl scene
  if (problem.constructor.name == "R3Problem") {
    problem_scene.update_r3(problem, gui_params.scene);
  } else if (problem.constructor.name == "Se2Problem") {
    problem_scene.update_se2(problem, gui_params.scene);
  } else {
    throw "Unknown class: " + problem.constructor.name;
  }

  // main scene
  renderer.setClearColor(0x000000, 0);

  renderer.setViewport(0, 0, window.innerWidth, window.innerHeight);

  gpuPanel.startQuery();
  renderer.render(problem_scene.scene, camera);
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

  gui.add(gui_params, "space", ["r3", "se2"]);
  gui.add(gui_params, "delay_before_restart", 0.1, 5, 0.1);

  for (const problem_type of ["r3_problem", "se2_problem"]) {
    const problem_folder = gui.addFolder(problem_type);
    const problem_params = gui_params[problem_type];
    if (problem_type == "se2_problem") {
      problem_folder.add(problem_params, "rho", 0.01, 1.5, 0.01);
    }
    const max_eta = { r3_problem: 1, se2_problem: 10 }[problem_type];
    problem_folder.add(problem_params, "eta", 0, max_eta, 0.01);
    problem_folder.add(problem_params, "max_num_obstacles", 0, 30, 1);
    problem_folder.add(problem_params, "obstacle_fraction", 0, 1, 0.01);
    const min_length = problem_folder.add(
      problem_params,
      "min_length",
      0.05,
      9.95,
      0.05
    );
    const max_length = problem_folder.add(
      problem_params,
      "max_length",
      0.1,
      10,
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
    problem_folder.add(problem_params, "goal_radius", 0.01, 2, 0.01);
    problem_folder.add(problem_params, "max_iterations", 0, 10000);
    problem_folder.add(problem_params, "iterations_per_frame", 1, 500, 1);
  }

  const scene_folder = gui.addFolder("scene");
  scene_folder.add(gui_params.scene, "rotate");
  scene_folder.add(gui_params.scene, "rotation_rate", 0, 1.5);
  scene_folder.add(gui_params.scene, "show_goal_region");
  scene_folder.add(gui_params.scene, "show_obstacles");
  scene_folder.add(gui_params.scene, "goal_region_opacity", 0, 1, 0.01);
  scene_folder.add(gui_params.scene, "obstacle_opacity", 0, 1, 0.01);
  scene_folder.add(gui_params.scene, "show_bounding_box");
  scene_folder.add(gui_params.scene, "bounding_box_opacity", 0, 1, 0.01);
  scene_folder.add(gui_params.scene, "show_axes");
}
