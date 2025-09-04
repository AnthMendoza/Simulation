import * as THREE from "three";

import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { OBJLoader } from "three/addons/loaders/OBJLoader.js";

let startTime = null; // Declare startTime here
let allValues = []; // Declare allValues outside the fetch block
const staticDuration = 0; 
const speed = 1;


// Create the scene
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x000000); // Light gray background 0xaaaaaa

// Set up the camera
const simulationViewPort = document.getElementById('simulationViewPort');
const canvas = document.getElementById('threeCanvas');
const camera = new THREE.PerspectiveCamera(96, window.innerWidth / window.innerHeight, 0.001, 100000);
const cameraOffset = new THREE.Vector3(30, 50, 10);  // Fixed offset relative to the object


// Create the renderer
const renderer = new THREE.WebGLRenderer({ canvas: canvas });
renderer.setSize(simulationViewPort.clientWidth, simulationViewPort.clientHeight);
camera.updateProjectionMatrix();

// Add lighting to the scene
const ambientLight = new THREE.AmbientLight(0x404040); // Soft light
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1.0); // Increase intensity
directionalLight.position.set(1, 1, 1).normalize();
scene.add(directionalLight);

// Variables to hold the object once loaded
let object;

const loader = new OBJLoader();
const GLTFloader = new GLTFLoader();

loader.load(
  '../static/3d/falcon.obj',
  function (loadedObject) {
    object = loadedObject;
    object.position.y = 1; // Start position of the object
    object.traverse(function (child) {
      if (child.isMesh) {
        child.material = new THREE.MeshStandardMaterial({ color: 0x637ea8 }); // Set a default color
      }
    });
    scene.add(object);
  },
  function (xhr) {
    console.log((xhr.loaded / xhr.total * 100) + '% loaded');
  },
  function (error) {
    console.log('An error occurred while loading the OBJ file:', error);
  }
);

// Create the floor
const floorGeometry = new THREE.PlaneGeometry(500, 500);
const floorMaterial = new THREE.MeshBasicMaterial({ color: 0x3f9b0b, side: THREE.DoubleSide });
const floor = new THREE.Mesh(floorGeometry, floorMaterial);

// Position and rotate the floor
floor.rotation.x = Math.PI / 2;  // Rotate the plane to lie flat on the XZ plane
floor.position.y = 0;  // Set the floor at y = 0

scene.add(floor);  // Add the floor to the scene

    
let  currentX = 0;
let  currentZ = 0;
let  currentY = 0;

let count = 1;

function updateObjectByTime(currentTime) {
  if (!object) return;  // Wait until the object is loaded

  while(count < data.VectorTimeStamp.length -2 && currentTime > parseFloat(data.VectorTimeStamp[count])){
    count +=1;
  }
  currentX = parseFloat(data.Xposition[count]);
  currentZ = parseFloat(data.Zposition[count]);
  currentY =  parseFloat(data.Yposition[count]);
  // Ensure count doesn't exceed bounds
  // Set the object's position and rotation
  object.position.set(currentX , currentZ , currentY);
  let directionVector = new THREE.Vector3(
    parseFloat(data.vehicleState0[count]),  
    parseFloat(data.vehicleState2[count]),  
    parseFloat(data.vehicleState1[count])
  );  
  const targetPosition = object.position.clone().add(directionVector);
  object.lookAt(targetPosition); 
  // Update camera to follow object with a static offset
  camera.position.copy(object.position).add(cameraOffset);
  camera.lookAt(object.position);
  const overlayText = document.getElementById('overlay-text');//+parseFloat(data.Zposition[0])).toFixed(1)
  overlayText.innerHTML = `Position( meters ):<br>X=${object.position.x.toFixed(1)}
                                              <br>Y=${(-object.position.z)}
                                              <br>altitude=${object.position.y.toFixed(1)} <br>
                                              <br> Velocity( m/s , mph )<br>${parseFloat(data.velocity[count]).toFixed(3)}
                                              <br>${(parseFloat(data.velocity[count])*2.237).toFixed(3)}<br>
                                              <br> Acceleration( G )${parseFloat(data.gForce[count]).toFixed(3)}`;
}



  // Animation loop
function animate(timestamp) {
  requestAnimationFrame(animate);
  
  if (startTime === null) startTime = timestamp;
  const elapsedTime = (timestamp - startTime) / 1000;  // time in seconds
  if (elapsedTime >= staticDuration) {
    updateObjectByTime(elapsedTime - staticDuration); // Subtract staticDuration from elapsed time
  }
  renderer.render(scene, camera);
}
animate(startTime);

  // Handle window resizing
  window.addEventListener('resize', () => {
    const simulationViewPort = document.getElementById('simulationViewPort');
    renderer.setSize(simulationViewPort.clientWidth, simulationViewPort.clientHeight);
    camera.aspect = simulationViewPort.clientWidth / simulationViewPort.clientHeight;
    camera.updateProjectionMatrix();
  });
