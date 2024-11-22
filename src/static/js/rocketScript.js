let startTime = null; // Declare startTime here
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

// Load the OBJ model
const loader = new THREE.OBJLoader();
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

const particleCount = 0; // Maximum number of particles
const particleGeometry = new THREE.BufferGeometry();
const particleMaterial = new THREE.PointsMaterial({ color: 0xaa4203, size: .3 });
const particlePositions = new Float32Array(particleCount * 3);
const particlesData = [];
const maxLifetime = 2;
const range = 3; 

const origin = new THREE.Vector3(currentX, currentZ, currentY); 


for (let i = 0; i < particleCount; i++) {
  const particle = {
    position: new THREE.Vector3(
      origin.x + (Math.random() - 0.5) * range, 
      origin.y + (Math.random() - 0.5) * range, 
      origin.z + (Math.random() - 0.5) * range
    ),
    direction: new THREE.Vector3(0,0,1),
    lifetime: Math.random() * maxLifetime 
  };
  particlesData.push(particle);
  

  particlePositions[i * 3] = particle.position.x;
  particlePositions[i * 3 + 1] = particle.position.y;
  particlePositions[i * 3 + 2] = particle.position.z;
}

particleGeometry.setAttribute('position', new THREE.BufferAttribute(particlePositions, 3));
const particles = new THREE.Points(particleGeometry, particleMaterial);
scene.add(particles);


function updateFire() {
  origin.x = currentX;
  origin.y = currentZ;
  origin.z = currentY;


  const positions = particles.geometry.attributes.position.array;
  

  for (let i = 0; i < particleCount; i++) {
      const particle = particlesData[i];

      particle.lifetime -= 0.016;

      if (particle.lifetime <= 0) {
          // Reset particle position relative to new origin position
          particle.position.set(
              origin.x + (Math.random() - 0.5) * range,
              origin.y + (Math.random() - 0.5) * range,
              origin.z + (Math.random() - 0.5) * range
          );
          particle.lifetime = maxLifetime;
      }

      particle.position.addScaledVector(particle.direction, speed);

      positions[i * 3] = particle.position.x;
      positions[i * 3 + 1] = particle.position.y;
      positions[i * 3 + 2] = particle.position.z;
  }

  particles.geometry.attributes.position.needsUpdate = true;
}



  // Animation loop
function animate(timestamp) {
  requestAnimationFrame(animate);
  updateFire();
  
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