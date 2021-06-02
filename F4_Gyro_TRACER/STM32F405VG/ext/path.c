#include "path.h"
#include "control.h"
#include "approx_math.h"


volatile Path path[2] = {{0}, {0}}; //Shadow memory
volatile Path path1[2] = {{0}, {0}}; //Shadow memory
volatile Path path2[2] = {{0}, {0}}; //Shadow memory
volatile Path path3[2] = {{0}, {0}}; //Shadow memory

//Path information
volatile bool next_path_required = false;
volatile s32 next_path_pos = 0;
volatile s32 next_path_max_v = 0;
volatile s32 next_path_acc = 0;
volatile u8 curr_path = 0;
volatile u8 pend_path = 0;
volatile bool path_static = false;

Path* gen_continuous_const_vel(const s32 vt, const s32 acc){
	return gen_const_vel(path[curr_path].tar_vel, path[curr_path].tar_pos, 
						path[curr_path].tar_vel_r, path[curr_path].tar_pos_r, 
						vt, acc);
}
/*
Path* gen_continuous_const_vel1(const s32 vt, const s32 acc){
	return gen_const_vel1(path1[curr_path].tar_vel, path1[curr_path].tar_pos, 
						path1[curr_path].tar_vel_r, path1[curr_path].tar_pos_r, 
						vt, acc);
}

Path* gen_continuous_const_vel2(const s32 vt, const s32 acc){
	return gen_const_vel2(path2[curr_path].tar_vel, path2[curr_path].tar_pos, 
						path2[curr_path].tar_vel_r, path2[curr_path].tar_pos_r, 
						vt, acc);
}

Path* gen_continuous_const_vel3(const s32 vt, const s32 acc){
	return gen_const_vel3(path3[curr_path].tar_vel, path3[curr_path].tar_pos, 
						path3[curr_path].tar_vel_r, path3[curr_path].tar_pos_r, 
						vt, acc);
}*/

Path* gen_const_vel(s32 v0, s32 s0, s32 vr, s32 sr, s32 vt, s32 acc){
	const u8 this_path = !pend_path;
	next_path_required = false;
	
	s32 nom_acc;
	if (vt > v0){
		nom_acc =  acc;
	}else{
		nom_acc = -acc;
	}
	
	u32 t1 = (s64)(vt - v0) * CONTROL_FREQ / nom_acc;
	s32 pt = ((s64)vt+v0)*t1/CONTROL_FREQ/2 + s0;
	
	#define path path[this_path]
	
	__disable_irq();
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	
	if (vt > 0){
		path.dir = DIR_POS;
	}else if(vt < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
	
	path.t1 = t1;
	path.t2 = t1;
	path.t3 = t1;
	path.vt = vt;
	path.ve = vt;
	path.end_pt = path.t1_pt = path.t2_pt = pt;
	path.itr = 0;
	
	pend_path = this_path;
	__enable_irq();
	#undef path
	
	return (Path*)&path[pend_path];
}
/*
Path* gen_const_vel1(s32 v0, s32 s0, s32 vr, s32 sr, s32 vt, s32 acc){
	const u8 this_path = !pend_path;
	next_path_required = false;
	
	s32 nom_acc;
	if (vt > v0){
		nom_acc =  acc;
	}else{
		nom_acc = -acc;
	}
	
	u32 t1 = (s64)(vt - v0) * CONTROL_FREQ / nom_acc;
	s32 pt = ((s64)vt+v0)*t1/CONTROL_FREQ/2 + s0;
	
	#define path path1[this_path]
	
	__disable_irq();
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	
	if (vt > 0){
		path.dir = DIR_POS;
	}else if(vt < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
	
	path.t1 = t1;
	path.t2 = t1;
	path.t3 = t1;
	path.vt = vt;
	path.ve = vt;
	path.end_pt = path.t1_pt = path.t2_pt = pt;
	path.itr = 0;
	
	pend_path = this_path;
	__enable_irq();
	#undef path
	
	return (Path*)&path1[pend_path];
}

Path* gen_const_vel2(s32 v0, s32 s0, s32 vr, s32 sr, s32 vt, s32 acc){
	const u8 this_path = !pend_path;
	next_path_required = false;
	
	s32 nom_acc;
	if (vt > v0){
		nom_acc =  acc;
	}else{
		nom_acc = -acc;
	}
	
	u32 t1 = (s64)(vt - v0) * CONTROL_FREQ / nom_acc;
	s32 pt = ((s64)vt+v0)*t1/CONTROL_FREQ/2 + s0;
	
	#define path path2[this_path]
	
	__disable_irq();
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	
	if (vt > 0){
		path.dir = DIR_POS;
	}else if(vt < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
	
	path.t1 = t1;
	path.t2 = t1;
	path.t3 = t1;
	path.vt = vt;
	path.ve = vt;
	path.end_pt = path.t1_pt = path.t2_pt = pt;
	path.itr = 0;
	
	pend_path = this_path;
	__enable_irq();
	#undef path
	
	return (Path*)&path2[pend_path];
}

Path* gen_const_vel3(s32 v0, s32 s0, s32 vr, s32 sr, s32 vt, s32 acc){
	const u8 this_path = !pend_path;
	next_path_required = false;
	
	s32 nom_acc;
	if (vt > v0){
		nom_acc =  acc;
	}else{
		nom_acc = -acc;
	}
	
	u32 t1 = (s64)(vt - v0) * CONTROL_FREQ / nom_acc;
	s32 pt = ((s64)vt+v0)*t1/CONTROL_FREQ/2 + s0;
	
	#define path path3[this_path]
	
	__disable_irq();
	path.tar_vel = v0;
	path.tar_pos = s0;
	path.tar_vel_r = vr;
	path.tar_pos_r = sr;
	
	path.nom_acc = nom_acc;
	path.seg_acc = nom_acc;
	
	if (vt > 0){
		path.dir = DIR_POS;
	}else if(vt < 0){
		path.dir = DIR_NEG;
	}else{
		path.dir = DIR_NEU;
	}
	
	path.t1 = t1;
	path.t2 = t1;
	path.t3 = t1;
	path.vt = vt;
	path.ve = vt;
	path.end_pt = path.t1_pt = path.t2_pt = pt;
	path.itr = 0;
	
	pend_path = this_path;
	__enable_irq();
	#undef path
	
	return (Path*)&path3[pend_path];
}*/

Path* path_iterate(){
	//Change to next path using shadow variable
	if (pend_path != curr_path){
		//Rough seas ahead, crew. Strap me to the mizzen when I give the word.
		curr_path = pend_path;
		path_static = false;
	}
	
	#define path path[curr_path]
	
	if (path.itr < path.t1){
		//Acceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel  += (path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos  += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		path.seg_acc = path.nom_acc;
		
	}else if(path.itr == path.t1){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t1_pt;
		path.tar_pos_r = 0;
		
		//pt_arrival_feedback(0);
	}
	
	if(path.itr >= path.t1 && path.itr < path.t2){
		//Constant phase
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		
	}else if(path.itr == path.t2){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t2_pt;
		path.tar_pos_r = 0;
		
		//pt_arrival_feedback(1);
	}
	
	if(path.itr >= path.t2 && path.itr < path.t3){
		//Deceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel += (-path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (-path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = -path.nom_acc;
		
	}else if(path.itr == path.t3){
		path.tar_pos = path.end_pt;
		path.tar_pos_r = 0;
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		
		path.seg_acc = 0;
		
		if (path.ve == 0){
			path.dir = DIR_NEU;
		}
		
		//Reached the end
		if (next_path_required){
			//gen_continuous_path(next_path_pos, next_path_max_v, next_path_acc);
		}else if (path.ve == 0){
			path_static = true;
		}
		
		//pt_arrival_feedback(2);
	}
	
	if (path.itr >= path.t3){
		//After the end
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		//Lock the iterator
		path.itr = path.t3 + 1;
	}
	path.itr++;
	#undef path
	
	return (Path*)&path[curr_path];
}
/*
Path* path_iterate1(){
	//Change to next path using shadow variable
	if (pend_path != curr_path){
		//Rough seas ahead, crew. Strap me to the mizzen when I give the word.
		curr_path = pend_path;
		path_static = false;
	}
	
	#define path path1[curr_path]
	
	if (path.itr < path.t1){
		//Acceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel  += (path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos  += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		path.seg_acc = path.nom_acc;
		
	}else if(path.itr == path.t1){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t1_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(0);
	}
	
	if(path.itr >= path.t1 && path.itr < path.t2){
		//Constant phase
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		
	}else if(path.itr == path.t2){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t2_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(1);
	}
	
	if(path.itr >= path.t2 && path.itr < path.t3){
		//Deceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel += (-path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (-path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = -path.nom_acc;
		
	}else if(path.itr == path.t3){
		path.tar_pos = path.end_pt;
		path.tar_pos_r = 0;
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		
		path.seg_acc = 0;
		
		if (path.ve == 0){
			path.dir = DIR_NEU;
		}
		
		//Reached the end
		if (next_path_required){
			//gen_continuous_path(next_path_pos, next_path_max_v, next_path_acc);
		}else if (path.ve == 0){
			path_static = true;
		}
		
		pt_arrival_feedback(2);
	}
	
	if (path.itr >= path.t3){
		//After the end
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		//Lock the iterator
		path.itr = path.t3 + 1;
	}
	path.itr++;
	#undef path
	
	return (Path*)&path1[curr_path];
}

Path* path_iterate2(){
	//Change to next path using shadow variable
	if (pend_path != curr_path){
		//Rough seas ahead, crew. Strap me to the mizzen when I give the word.
		curr_path = pend_path;
		path_static = false;
	}
	
	#define path path2[curr_path]
	
	if (path.itr < path.t1){
		//Acceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel  += (path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos  += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		path.seg_acc = path.nom_acc;
		
	}else if(path.itr == path.t1){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t1_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(0);
	}
	
	if(path.itr >= path.t1 && path.itr < path.t2){
		//Constant phase
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		
	}else if(path.itr == path.t2){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t2_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(1);
	}
	
	if(path.itr >= path.t2 && path.itr < path.t3){
		//Deceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel += (-path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (-path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = -path.nom_acc;
		
	}else if(path.itr == path.t3){
		path.tar_pos = path.end_pt;
		path.tar_pos_r = 0;
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		
		path.seg_acc = 0;
		
		if (path.ve == 0){
			path.dir = DIR_NEU;
		}
		
		//Reached the end
		if (next_path_required){
			//gen_continuous_path(next_path_pos, next_path_max_v, next_path_acc);
		}else if (path.ve == 0){
			path_static = true;
		}
		
		pt_arrival_feedback(2);
	}
	
	if (path.itr >= path.t3){
		//After the end
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		//Lock the iterator
		path.itr = path.t3 + 1;
	}
	path.itr++;
	#undef path
	
	return (Path*)&path2[curr_path];
}

Path* path_iterate3(){
	//Change to next path using shadow variable
	if (pend_path != curr_path){
		//Rough seas ahead, crew. Strap me to the mizzen when I give the word.
		curr_path = pend_path;
		path_static = false;
	}
	
	#define path path3[curr_path]
	
	if (path.itr < path.t1){
		//Acceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel  += (path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos  += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		path.seg_acc = path.nom_acc;
		
	}else if(path.itr == path.t1){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t1_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(0);
	}
	
	if(path.itr >= path.t1 && path.itr < path.t2){
		//Constant phase
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		
	}else if(path.itr == path.t2){
		//Recali to reduce integration error
		path.tar_vel = path.vt;
		path.tar_vel_r = 0;
		path.tar_pos = path.t2_pt;
		path.tar_pos_r = 0;
		
		pt_arrival_feedback(1);
	}
	
	if(path.itr >= path.t2 && path.itr < path.t3){
		//Deceleration phase
		const s32 orig_vel = path.tar_vel;
		
		path.tar_vel += (-path.nom_acc + path.tar_vel_r) / CONTROL_FREQ;
		path.tar_vel_r = (-path.nom_acc + path.tar_vel_r) % CONTROL_FREQ;
		
		//Trapezoidal Rule
		const s32 temp = (orig_vel + path.tar_vel) + path.tar_pos_r;
		path.tar_pos += temp / (CONTROL_FREQ*2);
		path.tar_pos_r = temp % (CONTROL_FREQ*2);
		
		path.seg_acc = -path.nom_acc;
		
	}else if(path.itr == path.t3){
		path.tar_pos = path.end_pt;
		path.tar_pos_r = 0;
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		
		path.seg_acc = 0;
		
		if (path.ve == 0){
			path.dir = DIR_NEU;
		}
		
		//Reached the end
		if (next_path_required){
			//gen_continuous_path(next_path_pos, next_path_max_v, next_path_acc);
		}else if (path.ve == 0){
			path_static = true;
		}
		
		pt_arrival_feedback(2);
	}
	
	if (path.itr >= path.t3){
		//After the end
		path.tar_vel = path.ve;
		path.tar_vel_r = 0;
		path.tar_pos += (path.tar_vel + path.tar_pos_r) / CONTROL_FREQ;
		path.tar_pos_r = (path.tar_vel + path.tar_pos_r) % CONTROL_FREQ;
		
		path.seg_acc = 0;
		//Lock the iterator
		path.itr = path.t3 + 1;
	}
	path.itr++;
	#undef path
	
	return (Path*)&path3[curr_path];
}*/

//Reset the path, lock the motor in position
void path_reset(){
	__disable_irq();
	const u8 this_path = !pend_path;
	path[this_path].t1 = path[this_path].t2 = path[this_path].t3 = 0;
	path[this_path].itr = 1;
	path[this_path].tar_pos = get_cnt();
	path[this_path].t1_pt = path[this_path].t2_pt = path[this_path].end_pt = path[this_path].tar_pos;
	path[this_path].tar_vel = path[this_path].tar_vel_r = path[this_path].tar_pos_r = 0;
	path[this_path].vt = path[this_path].ve = 0;
	path[this_path].dir = DIR_NEU;
	path[this_path].nom_acc = path[this_path].seg_acc = MAX_ORIG_ACC;
	path_static = true;
	curr_path = this_path;
	__enable_irq();
}

//To check if the path has finished running or not [That it will not change velocity anymore]
bool is_path_running(){
	return path[curr_path].itr < path[curr_path].t3;
}

u8 get_curr_path(){
	return curr_path;
}

s32 get_path_vel_scaled(){
	return path[curr_path].tar_vel*CONTROL_FREQ + path[curr_path].tar_vel_r;
}

/*s64 get_path_pos_scaled(){
	return (s64)path[curr_path].tar_pos*(s64)CONTROL_FREQ + path[curr_path].tar_pos_r;
}*/

s32 get_path_vel(){
	return path[curr_path].tar_vel;
}

s32 get_path_pos(){
	return path[curr_path].tar_pos;
}

u32 get_t1(){
	return path[curr_path].t1;
}

u32 get_t2(){
	return path[curr_path].t2;
}

u32 get_t3(){
	return path[curr_path].t3;
}

s32 get_vt(){
	return path[curr_path].vt;
}

s32 get_ve(){
	return path[curr_path].ve;
}

u32 get_itr(){
	return path[curr_path].itr;
}

u8 get_path_dir(){
	return path[curr_path].dir;
}

bool get_next_required(){
	return next_path_required;
}

bool is_path_static(){
	return path_static;
}

