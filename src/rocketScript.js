
let startTime = null; // Declare startTime here
let allValues = []; // Declare allValues outside the fetch block
  const staticDuration = 0; 

fetch('../data.csv')
  .then(response => response.text())
  .then(data => {
      const rows = data.split('\n');
      const headers = rows[0].split(',').map(header => header.trim());
      const result = rows.slice(1).map(row => {
          const values = row.split(',').map(value => value.trim());
          return headers.reduce((obj, header, index) => {
              obj[header] = values[index] ? values[index] : null; // Handle empty values
              return obj;
          }, {});
      });

      console.log(result); // Output the array of objects

      // Retrieving all column values
      allValues = headers.map(header => {
          return result.map(row => row[header]);
      });

      console.log(allValues); // Output all values from each column
  })
  .catch(error => {
      console.error('Error fetching CSV:', error);
  });


  // Create the scene
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x000000); // Light gray background 0xaaaaaa

  // Set up the camera
  const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
  const cameraOffset = new THREE.Vector3(30, 50, 10);  // Fixed offset relative to the object


  // Create the renderer
  const renderer = new THREE.WebGLRenderer();
  renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(renderer.domElement);

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
    '../3d/falcon.obj',  // Replace with your .obj file path
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
  const floorGeometry = new THREE.PlaneGeometry(500, 500);  // Size of the floor (50x50 units)
  const floorMaterial = new THREE.MeshBasicMaterial({ color: 0x3f9b0b, side: THREE.DoubleSide });
  const floor = new THREE.Mesh(floorGeometry, floorMaterial);

  // Position and rotate the floor
  floor.rotation.x = Math.PI / 2;  // Rotate the plane to lie flat on the XZ plane
  floor.position.y = 0;  // Set the floor at y = 0

  scene.add(floor);  // Add the floor to the scene

  let count = 0;
  function updateObjectByTime(currentTime) {
    if (!object) return;  // Wait until the object is loaded
    
    while(count < allValues[0].length -2 && currentTime > parseFloat(allValues[0][count])){
      count +=1;
    }
    
    // Ensure count doesn't exceed bounds

    // Set the object's position and rotation
    object.position.set(
      parseFloat(allValues[1][count]), 
      parseFloat(allValues[3][count]), 
      parseFloat(allValues[2][count]-269.7)
    );
    console.log(parseFloat(allValues[3][count]) )
    let directionVector = new THREE.Vector3(
      parseFloat(allValues[4][count]),  
      parseFloat(allValues[6][count]),  
      parseFloat(allValues[5][count])
    );  
    const targetPosition = object.position.clone().add(directionVector);
    object.lookAt(targetPosition); 

    // Update camera to follow object with a static offset
    camera.position.copy(object.position).add(cameraOffset);
    camera.lookAt(object.position);

    const overlayText = document.getElementById('overlay-text');
    overlayText.innerHTML = `Position( meters ):<br>X=${object.position.x.toFixed(1)}
                                                <br>Y=${(-object.position.z+parseFloat(allValues[2][0])).toFixed(1)}
                                                <br>altitude=${object.position.y.toFixed(1)} <br>
                                                <br> Velocity( m/s , mph )<br>${parseFloat(allValues[7][count]).toFixed(3)}
                                                <br>${(parseFloat(allValues[7][count])*2.237).toFixed(3)}<br>
                                                <br> Acceleration( G )${parseFloat(allValues[8][count]).toFixed(3)}`;
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
    const width = window.innerWidth;
    const height = window.innerHeight;
    renderer.setSize(width, height);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  });