// Galton Board


class Objects {
  
  float x, y;
  float v_x, v_y;
  float a_x, a_y;
  float mass;
  float dt;
  
  Objects(float x, float y, float v_x, float v_y, float a_x, float a_y, float mass) {
    this.x = x;
    this.y = y;
    this.v_x = v_x;
    this.v_y = v_y;
    this.a_x = a_x;
    this.a_y = a_y;
    this.mass = mass;
  }
  
  void update_velocity() {
    v_x += a_x*dt;
    v_y += a_y*dt;
  }
  void update_position() {
    x += v_x*dt;
    y += v_y*dt;
  }
  
  void update() {
    dt = 1/frameRate; 
    update_velocity();
    update_position();
    //println(x, y);
    
  }
  
}

class MBall extends Objects {
  float dia, new_v_x, new_v_y;
  MBall (float x, float y, float v_x, float v_y, float a_x, float a_y, float dia, float mass) {
    super(x, y, v_x, v_y, a_x, a_y, mass);
    this.dia = dia;
    this.new_v_x = 9000000;
    this.new_v_y = 9000000;
  }

  float vdist(float x1, float x2, float y1, float y2) {
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
  }

  
  boolean collision_detection_mball (MBall obj2) {
    if (vdist(this.x, obj2.x, this.y, obj2.y) <= ((this.dia/2.0) + (obj2.dia/2.0))) return true;
    return false;
  }
  
  boolean collision_detection_fball (FBall obj2) {
    if (vdist(this.x, obj2.x, this.y, obj2.y) <= ((this.dia/2.0) + (obj2.dia/2.0))) return true;
    return false;
  }
  boolean collision_detection_walls (Wall  obj2) {
    float circ_left = this.x - this.dia/2; // left
    float circ_right = this.x + this.dia/2; // right
    float circ_top = this.y + this.dia/2; // top
    float circ_bottom = this.y - this.dia/2; //bottom
    
    float wall_left = obj2.x - obj2.w/2; // left
    float wall_right = obj2.x + obj2.w/2; // right
    float wall_top = obj2.y + obj2.h/2; //top
    float wall_bottom = obj2.y - obj2.h/2; //bottom
    if ((circ_left <= wall_right && circ_right >= wall_left) && (circ_top >= wall_bottom && circ_bottom <= wall_top)) return true;
    return false;
  }
  
 


  void elastic_collision_ball_ball (Objects obj2, float obj2_dia) {
    float scalar = (2*obj2.mass)/(this.mass + obj2.mass);
    //scalar = 2;
    //println(scalar, this.x, this.y, obj2.x, obj2.y);

    float C1mC2_x = this.x - obj2.x;
    float C1mC2_y = this.y - obj2.y;
    //println(scalar, this.x, this.y, obj2.x, obj2.y, C1mC2_x, C1mC2_y, (pow(C1mC2_x, 2) + pow(C1mC2_y, y))) ;
    scalar = scalar/(pow(C1mC2_x, 2) + pow(C1mC2_y, 2));
    //println(scalar);

    float v1mv2_x = this.v_x - obj2.v_x;
    float v1mv2_y = this.v_y - obj2.v_y;

    float diff_v_dot_diff_C = C1mC2_x*v1mv2_x + C1mC2_y*v1mv2_y;

    scalar = scalar*diff_v_dot_diff_C;

    //println(scalar);

    float diff_v_x = C1mC2_x*scalar;
    float diff_v_y = C1mC2_y*scalar;

    this.v_x = this.v_x - diff_v_x;
    this.v_y = this.v_y - diff_v_y;


    // Damping factor
    float damping_factor = 0.9;
    this.new_v_x = damping_factor*this.v_x;
    this.new_v_y = damping_factor*this.v_y;


    // Pushback logic
    float dist_C1_C2 = sqrt((pow(C1mC2_x, 2) + pow(C1mC2_y, 2)));
    if (dist_C1_C2 < (this.dia/2.0) + (obj2_dia/2.0)) {
      float diff = (this.dia/2.0) + (obj2_dia/2.0) - dist_C1_C2;
      diff = diff/dist_C1_C2;
      this.x += C1mC2_x*diff;
      this.y += C1mC2_y*diff;
    }
    //super.update();
  }


  void elastic_collision_ball_wall (Wall obj2) {
    //Assume Elastic Collision
    // Check the closest point to the obj center
    float circ_left = this.x - this.dia/2; // left
    float circ_right = this.x + this.dia/2; // right
    float circ_top = this.y + this.dia/2; // top
    float circ_bottom = this.y - this.dia/2; //bottom
    
    float damping_factor = 0.6;

    float wall_left = obj2.x - obj2.w/2; // left
    float wall_right = obj2.x + obj2.w/2; // right
    float wall_top = obj2.y + obj2.h/2; //top
    float wall_bottom = obj2.y - obj2.h/2; //bottom

    // Left Reflection
    if (circ_right >= wall_left && circ_left < wall_left && this.y < wall_top && this.y > wall_bottom) {
      this.new_v_x = this.v_x*(-1*damping_factor);
      this.new_v_y = this.v_y;
      // Pushback
      this.x -= (circ_right - wall_left);
    }
    // Right Reflection
    else if (circ_left <= wall_right && circ_right > wall_right && this.y < wall_top && this.y > wall_bottom) {
      this.new_v_x = this.v_x*(-1*damping_factor);
      this.new_v_y = this.v_y;
      // Push Back
      this.x += (wall_right - circ_left);
    }
    // Top Reflection
    else if (circ_bottom <= wall_top && circ_top > wall_top && this.x < wall_right && this.x > wall_right) {
      this.new_v_y = this.v_y*(-1*damping_factor);
      this.new_v_x = this.v_x;
      // Push Back
      this.y += (wall_top - circ_bottom);
    }
    // Bottom Reflection 
    else {
      this.new_v_y = this.v_y*(-1*damping_factor);
      this.new_v_x = this.v_x;
      // Push Back
      this.y -= (circ_top - wall_bottom);
    }
    //super.update();

  } 
  
  void update() {
      if(this.new_v_x != 9000000 && this.new_v_y != 9000000) {  
        this.v_x = this.new_v_x;
        this.v_y = this.new_v_y;
        this.new_v_x = 9000000;
        this.new_v_y = 9000000;
      }
      //println(this.x, this.y, this.dia);
      super.update();
      //println(this.x, this.y, this.dia);

      noStroke();
      fill(127, 255, 127);
      circle(this.x, this.y, this.dia);
  }

}


class FBall extends Objects {
  float dia;
  FBall (float x, float y, float dia, float mass) {
    super(x, y, 0, 0, 0, 0, mass);
    this.dia = dia;
  }
  
  void update() {
      super.update();
      
      noStroke();
      fill(255);
      circle(this.x, this.y, this.dia);
  }
  
}


class Wall extends Objects {
  float w,h;   
  Wall (float x, float y, float w, float h, float mass) {
    super(x, y, 0, 0, 0, 0, mass);
    this.w = w;
    this.h = h;
  }
  
  void update() {
    super.update();
    noStroke();
    fill(255);
    rectMode(CENTER);
    rect(this.x, this.y, this.w, this.h);
  }

}






float gravity;
MBall [] mballs;
FBall [] fballs;
Wall [] walls;
Wall floor;
Wall left_wall;
Wall right_wall;
Wall ceiling;
float fixed_num;

boolean debounce;
int total_m_balls;
int max_m_balls;

float ball_start;
float ball_end;
float ball_mass;
float ball_dia;
float ball_height;

void setup() {
  size(500, 1000);
  //surface.setResizable(true);
  gravity = 100;

  // Initialize objects in correct locations

  //float ball_height = 0;
  //float ball_rand_pos;
  ball_start = width*0.45;
  ball_end = width*0.55;
  ball_mass = 50;
  ball_dia = 20;
  total_m_balls = 0;
  max_m_balls = 1000;
  ball_height = 0.0;
  int i;

  // TODO: Try manually adding balls. Also try adding walls. 
  mballs = new MBall[max_m_balls];

  //for (int n = 0; n < mballs.length; n++) {
  //  float ball_rand_pos = random(ball_start, ball_end);
  //  mballs[i++] = new MBall(ball_rand_pos, ball_height, 0, 0, 0, gravity, ball_dia, ball_mass);
  //  ball_height -= ball_dia*1.5;
  //}
  debounce = false;
  
  
  // Fixed balls
  float fixed_start_x = 0;
  float fixed_start_y = 100;
  float fixed_dia = ball_dia/2;
  float fixed_delta = 4*fixed_dia;
  
  float fixed_mass = 100;
  
  int num_fixed_rows = 10;
  int num_fixed_cols = int(width/fixed_delta) + 1;
  
  i = 0;
  fballs = new FBall[num_fixed_rows * num_fixed_cols];
  fixed_num = 0;
  int skip = 0;
  //int skip = num_fixed_cols/2;
  for (int n = 0; n < num_fixed_rows; n++) {
    
    for (int m = 0 + skip; m < num_fixed_cols - skip; m++) {
      fballs[i++] = new FBall(fixed_start_x + m * fixed_delta  + (n % 2) * (fixed_delta / 2), fixed_start_y + n * fixed_delta, fixed_dia, fixed_mass);
      fixed_num+=1;
    }
    //skip--;
  }

  // Floor
  float floor_mass = 10;
  float floor_height = ball_dia*2;
  floor = new Wall(width/2, height, width , floor_height, floor_mass);
  
  // Walls
  float wall_width = ball_dia*0.5;
  float wall_delta = 4*ball_dia;
  
  float wall_start = 0;
  
  float wall_mass = 10;
  
  float wall_height = 900;
  int num_walls = int(width/wall_width);

  walls = new Wall[num_walls];
  for (int n = 0; n < num_walls; n++) {
    walls[n] = new Wall(wall_start + n * wall_delta, height - floor_height, wall_width, wall_height, wall_mass); // Slightly above all ball spawn location
  }

  //Right Wall
  right_wall = new Wall(width+ball_dia*2, height/2, ball_dia*4, height-ball_height-ball_dia*1.5, wall_mass);
  //Left Wall
  left_wall = new Wall(-ball_dia*2, height/2, ball_dia*4, height-ball_height-ball_dia*1.5, wall_mass);
  // Ceiling 
  ceiling = new Wall(width/2, ball_height-ball_dia*1.5, width, floor_height, floor_mass);// Slightly above all ball spawn location
}

void draw() {
  background(0);
    // Add balls
  if (mousePressed == true) {
    if(!debounce) {
      if(total_m_balls < max_m_balls) {
        //println("OK");
        float random_loc = random(ball_start, ball_end);
        mballs[total_m_balls] = new MBall(random_loc, ball_height, 0, 0, 0, gravity, ball_dia, ball_mass);
        //println(total_m_balls, ":", mballs[total_m_balls].x, mballs[total_m_balls].y, mballs[total_m_balls].v_x, mballs[total_m_balls].v_y, mballs[total_m_balls].a_x, mballs[total_m_balls].a_y, mballs[total_m_balls].dia, mballs[total_m_balls].mass);
        //println("total_m_balls", total_m_balls);
        total_m_balls+=1;
        debounce = true;
      }
    }
  }
  else {
    debounce = false;
  }

  // Call Updates
  for (int n = 0; n < total_m_balls; n++) {
    //println(n);
    //println(n, ":", mballs[n].x, mballs[n].y, mballs[n].v_x, mballs[n].v_y, mballs[n].a_x, mballs[n].a_y, mballs[n].dia, mballs[n].mass);
    mballs[n].update();
  }

  for (int n = 0; n < fixed_num; n++) {
    fballs[n].update();
  }
  
  for (int n = 0; n < walls.length; n++) {
    walls[n].update();
  }
  
  floor.update();
  right_wall.update();
  left_wall.update();
  ceiling.update();



  // Call Check Detection and Elastic Collisions
  int i, j;
  // Mball to Mball Collision
  for (i = 0; i < total_m_balls; i++) {
    for(j = 0; j < total_m_balls; j++) {
      if(i != j) {
        if(mballs[i].collision_detection_mball(mballs[j])) mballs[i].elastic_collision_ball_ball(mballs[j], mballs[j].dia);
      }
    }
  }

  // Mball to Fball Collision
  for (i = 0; i < total_m_balls; i++) {
    for(j = 0; j < fballs.length; j++) {
      if(mballs[i].collision_detection_fball(fballs[j])) mballs[i].elastic_collision_ball_ball(fballs[j], fballs[j].dia);
    }
  }
  
  // Mball to Floor Collision
  for (i = 0; i < total_m_balls; i++) {
    if(mballs[i].collision_detection_walls(floor)) mballs[i].elastic_collision_ball_wall(floor); //println("OK");}
  }

  // Mball to Walls Collision
  for (i = 0; i < total_m_balls; i++) {
    for(j = 0; j < walls.length; j++) {
      if(mballs[i].collision_detection_walls(walls[j])) mballs[i].elastic_collision_ball_wall(walls[j]); //println("OK");}
    }
  }

  // Mball to rightWall Collision
  for (i = 0; i < total_m_balls; i++) {
    if(mballs[i].collision_detection_walls(right_wall)) mballs[i].elastic_collision_ball_wall(right_wall); //println("OK");}
  }

  //Mball to leftWall Collision
  for (i = 0; i < total_m_balls; i++) {
    if(mballs[i].collision_detection_walls(left_wall)) mballs[i].elastic_collision_ball_wall(left_wall); //println("OK");}
  }

  //Mball to ceiling Collision
  for (i = 0; i < total_m_balls; i++) {
    if(mballs[i].collision_detection_walls(ceiling)) mballs[i].elastic_collision_ball_wall(ceiling); //println("OK");}
  }
    
}
