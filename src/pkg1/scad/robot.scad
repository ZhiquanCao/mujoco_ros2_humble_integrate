
// color("white")
// translate([-80,0,70])
// rotate([0,90,0])
// rotate([0,0,90])
// import("6DOF.stl", convexity=3);



module sequentialHull(){
    for (i = [0: $children-2])
        hull(){
            children(i);
            children(i+1);
        }
}	


module LX16A(){


	color("white")
	cube([30,47,25],center=true);

	color("grey")
	translate([-25,13,0])
	rotate([0,90,0])
	cylinder(h=45, r=9, center=false, $fn=20);
}


module batt(){
	translate([0,0,-10])
	cube([70,40,103],center=true);
}

module foot(){
	difference(){

		rotate([90,0,0])
		{

		cylinder(r=7.4,h=5.2,center = true,$fn = 40);
		translate([25,0,0])
		cylinder(r=7.4,h=5.2,center = true,$fn = 40);
		}

		rotate([90,0,0])
		{

		cylinder(r=5.7/2,h=25.2,center = true,$fn = 40);
		translate([25,0,0])
		cylinder(r=5.7/2,h=25.2,center = true,$fn = 40);
		}


	}


	difference(){

		union(){

			translate([30,0,-16])
			rotate([0,0,90])
			cube([22,77,6],center=true);
			// base_bracket();
			// servo();
			translate([25,0,-10])
			rotate([0,90,90])
			cube([15,77,1.5],center=true);

			for(i = [1:3:20]){
				translate([25,0,2 - i])
				rotate([0,0,90])
				cube([14+i/3,67,1.5],center=true);
			}

			translate([11,0,-3])
			cube([22,15,6],center=true);
		}

		translate([48,0,3])
		rotate([0,17,0])
		cube([40,30,20],center=true);


		translate([-15,0,-5])
		rotate([0,-60,0])
		cube([40,30,20],center=true);

		rotate([90,0,0])
		{

			cylinder(r=7.2,h=25.2,center = true,$fn = 40);
			translate([23,0,0])
			cylinder(r=7.2,h=25.2,center = true,$fn = 40);
		}

	}

}


module compute(){
	translate([0,0,50])
	cube([100,100,20],center=true);
	translate([-40,0,20])
	cube([40,40,60],center=true);
}



module left_ankle() {
	translate([-27.5,49.5,-299.5])
	foot();	
}

module left_shin(){
	translate([0,26,-169])
{
	sequentialHull()
	{
		translate([0,45,0])
		rotate([90,0,0])
		cylinder(h=3,r=10,center=true);

		translate([0,42,-40])
		cylinder(h=3,r=5,center=true);
		translate([0,22,-130])
		rotate([90,0,0])
		cylinder(h=3,r=7,center=true);
		translate([0,2,-40])
		cylinder(h=3,r=5,center=true);


		rotate([90,0,0])
		cylinder(h=3,r=10,center=true);
	}
}


}

module left_shin_link(){
	translate([-30,26,-169])
	{
		sequentialHull()
		{
			translate([0,22,-130])
			rotate([90,0,0])
			cylinder(h=3,r=7,center=true);
			translate([0,7,-40])
			cylinder(h=3,r=2,center=true);

			translate([0,7,-1])
			rotate([90,0,0])
			cylinder(h=3,r=10,center=true);
		}
	}	
}


module left_thigh(){
	translate([0,30,-9])
	{
		sequentialHull()
		{
			translate([0,42,0])
			rotate([90,0,0])
			cylinder(h=3,r=10,center=true);

			translate([0,42,-40])
			cylinder(h=3,r=5,center=true);
			translate([0,22,-130])
			rotate([90,0,0])
			cylinder(h=3,r=7,center=true);
			translate([0,2,-40])
			cylinder(h=3,r=5,center=true);


			rotate([90,0,0])
			cylinder(h=3,r=10,center=true);
		}
	}

	translate([-2.5,55,-156])
	rotate([0,0,90])
	rotate([-90,0,0])
	LX16A();

}
module left_hip() {
		
	translate([-0,55,2])
	rotate([0,0,90])
	rotate([-90,0,0])
	// rotate([180,0,0])
	LX16A();

}
module left_pelvis(){
	translate([-40,45,0])
	LX16A();
}
module left_leg()
{
	left_pelvis();
	left_hip();
	left_thigh();
	left_shin_link();
	left_shin();
	left_ankle();
}
module body(){
	// batt();
	// compute();
	left_leg();
	mirror([0,1,0])
	left_leg();
}

translate([0,57.5,0])
body();





// // translate([-20,-60,0])
// // rotate([0,90,0])
// // cylinder(h=10, r=7, center=false, $fn=20);

// translate([-40,-45,0])
// rotate([180,0,0])
// LX16A();


// translate([-0,-55,2])
// rotate([0,0,-90])
// rotate([-90,0,0])
// // rotate([180,0,0])
// LX16A();



// translate([-2.5,-55,-156])

// // translate([-0,-55,2])
// rotate([0,0,-90])
// rotate([-90,0,0])
// // rotate([180,0,0])
// LX16A();


// translate([0,-46,0])





// 


