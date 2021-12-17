numAsteroids = 20;
earth = {};
ufo = {};
asteroids = [];

const minX = -5, maxX = 5;
const minY = -3, maxY = 3;

// ============================================
// Initialization
// ============================================

// Initialize Earth.
// Earth is not physically simulated, therefore
// we don't need to include all the info we include
// in the ufo and the asteroids.
function initEarth()
{
  return {
    position: {x: 3.5, y: 1.5},
    velocity: {x: 0, y: 0},
    size: 0.5,
  };
}

// Init Ufo
function initUfo()
{
  return {
    position: {x: -3.5, y: 0.0},
    velocity: {x: 0, y: 0},
    force: {x: 0.0, y: 0.0},
    alpha: 0.0,
    angularVelocity: 0.0, 
    torque: 0.0,
    mass: Math.PI * 0.3 * 0.3 * 5, 
    MoI: Math.PI * 0.5 * 0.3 * 0.3 * 0.3 * 0.3,
    size: 0.3,
  };
}

// Init Asteroids
// We randomly initialize the properties
// so that we find ourselves in a interesting
// environment.
function initAsteroids()
{
  var density = 30.0;
  var minSize = 0.08, maxSize = 0.3;

  var initAsteroids = [];
  for(var i = 0; i < numAsteroids; i++)
  {
    var randX = Math.random() * (maxX - minX) + minX;
    var randY = Math.random() * (maxY - minY) + minY;
    var randVelX = Math.random() * 0.5 - 0.25; randVelX += Math.sign(randVelX) * 0.25;
    var randVelY = Math.random() * 0.5 - 0.25; randVelY += Math.sign(randVelY) * 0.25;
    var randSize = Math.random() * (maxSize - minSize) + minSize;
    var currentAsteroid = {
      position: {x: randX, y: randY}, 
      velocity: {x: randVelX, y: randVelY}, 
      force: {x: 0.0, y: 0.0},
      alpha: 0.0,
      angularVelocity: Math.random() - 0.5, 
      torque: 0.0,
      mass: density * randSize * randSize * Math.PI,
      MoI: Math.PI * 0.5 * randSize * randSize * randSize * randSize,
      size:randSize
    };
    initAsteroids.push(currentAsteroid);
  }
  return initAsteroids;
}

// Call all initialization functions.
function initScene()
{
  earth = initEarth();
  ufo = initUfo();
  asteroids = initAsteroids();
}

// ============================================
// Physics Simulation
// ============================================

// This code is in charge of updating the positions, orientations,
// linear velocities, and angular velocities for a sphere.
// We use symplectic euler.
function integrate(sphere, dt)
{
  // Task 1

  // Linear
  sphere.velocity.x += sphere.force.x / sphere.mass * dt
  sphere.velocity.y += sphere.force.y / sphere.mass * dt
  sphere.position.x += sphere.velocity.x * dt
  sphere.position.y += sphere.velocity.y * dt

  // Angular
  sphere.angularVelocity += sphere.torque / sphere.MoI * dt
  sphere.alpha += sphere.angularVelocity * dt
}

function dot_2d(v1,v2) {
  return v1[0] * v2[0] + v1[1] * v2[1]
}

// Collision handling for two 2D sphere objects. We limit ourselves to linear impulses.
// First: check if the two object are colliding (intersecting). If not, return.
// Second: Project the objects such that there is no more interpenetration between the two.
// Third: Compute the impulse and the relative change in velocities for the two bodies.
function sphereSphereCollisionHandling(sphereA, sphereB)
{
  // Check if collision
  // ##################
  distx = sphereA.position.x - sphereB.position.x
  disty = sphereA.position.y - sphereB.position.y
  dist = Math.sqrt(distx*distx + disty*disty)
  if (dist > sphereA.size + sphereB.size) 
    return

  // Project to avoid overlap
  // #############################
  normx = distx / dist
  normy = disty / dist
  overlap = sphereA.size + sphereB.size - dist
  sphereA.position.x += normx * overlap
  sphereA.position.y += normy * overlap


  // Compute Impulse
  // ###############
  epsilon = 0.9 // bounciness
  normal = [normx, normy]
  vela_minues_velb = [sphereA.velocity.x - sphereB.velocity.x, sphereA.velocity.y - sphereB.velocity.y]
  J = -(1+epsilon) * dot_2d(vela_minues_velb, normal) / (dot_2d(normal, normal) * (1/sphereA.mass + 1/sphereB.mass)) 
  
  // Update Linear
  // #############
  sphereA.velocity.x += J * normx / sphereA.mass
  sphereA.velocity.y += J * normy / sphereA.mass
  sphereB.velocity.x -= J * normx  / sphereB.mass
  sphereB.velocity.y -= J * normy  / sphereB.mass
}

// If an object goes out of screen, it brings it back to the visible space.
function projectObjects()
{
  // Ufo
  const posX = ufo.position.x;
  const posY = ufo.position.y;
  
  if(posX < minX)
  {
    ufo = initUfo();
  }

  if(posX > maxX)
  {
    ufo = initUfo();
  }

  if(posY < minY)
  {
    ufo = initUfo();
  }

  if(posY > maxY)
  {
    ufo = initUfo();
  }

  // Asteroids
  for(var i = 0; i < numAsteroids; i++)
  {
    const posX = asteroids[i].position.x;
    const posY = asteroids[i].position.y;
    
    if(posX < minX)
    {
      asteroids[i].position.x = maxX; 
    }

    if(posX > maxX)
    {
      asteroids[i].position.x = minX; 
    }

    if(posY < minY)
    {
      asteroids[i].position.y = maxY; 
    }

    if(posY > maxY)
    {
      asteroids[i].position.y = minY; 
    }
  }
}

// Call all the necessary methods to update the full physics step.
function performSimStep(dt)
{
  // First perform time integration
  // for all dynamic objects.
  integrate(ufo, dt);
  for(var i = 0; i < numAsteroids; i++)
  {
    integrate(asteroids[i], dt);
  }

  // Resolve Collisions
  for(var i = 0; i < numAsteroids; i++)
  {
    sphereSphereCollisionHandling(asteroids[i], ufo);
  }
  for(var i = 0; i < numAsteroids-1; i++)
  {
    sphereSphereCollisionHandling(asteroids[i], ufo);
    for(var j = i + 1; j < numAsteroids; j++)
    {
      sphereSphereCollisionHandling(asteroids[i], asteroids[j]);
    }
  }

  // Project objects back to the visible space.
  projectObjects();
}