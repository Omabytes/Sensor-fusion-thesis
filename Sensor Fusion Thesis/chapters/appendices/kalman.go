/* Port of Kalman Filter from MATLAB to Go */

package main

import("fmt"
	   "math"
	   "github.com/gonum/matrix/mat64")

func main(){
	//Declare
	var duration float64 = 0.1
	var dt = 0.1
	var t float64			//time
	var Pt mat64.Dense		//p_{t|t-1} estimation covariance
	var K mat64.Dense		//Kalman gain matrix
	var Inn mat64.Dense		//Innovation vector
	var InnE mat64.Dense		//Innovation covariance
	var a mat64.Dense		//temp variable a
	var b mat64.Dense		//temp variable b
	var c mat64.Dense		//temp variable c
	var d mat64.Dense		//temp variable d
	var e mat64.Dense		//temp variable e
	var f mat64.Dense		//temp variable f
	var g mat64.Dense		//temp variable g
	var h mat64.Dense		//temp variable h
	var i mat64.Dense		//temp variable i
	var j mat64.Dense		//temp variable j

	
	//Intialise
	sensnoise := mat64.NewDense(6,1,[]float64{0,0,0,0,0,0})	//sensor noise
	R := mat64.NewDense(6,6,[]float64{
		math.Pow(sensnoise.At(0,0),2),0,0,0,0,0,
		0,math.Pow(sensnoise.At(1,0),2),0,0,0,0,
		0,0,math.Pow(sensnoise.At(2,0),2),0,0,0,
		0,0,0,math.Pow(sensnoise.At(3,0),2),0,0,
		0,0,0,0,math.Pow(sensnoise.At(4,0),2),0,
		0,0,0,0,0,math.Pow(sensnoise.At(5,0),2),
		})	//measurement error covariance 
	extnoise := mat64.NewDense(6,1,[]float64{0,0,0,0,0,0})	//external noise
	F := mat64.NewDense(9,9,[]float64{
		1,0,0,dt,0,0,0,0,0,
		0,1,0,0,dt,0,0,0,0,
		0,0,1,0,0,dt,0,0,0,
		0,0,0,1,0,0,0,0,0,
		0,0,0,0,1,0,0,0,0,
		0,0,0,0,0,1,0,0,0,
		0,0,0,0,0,0,1,0,0,
		0,0,0,0,0,0,0,1,0,
		0,0,0,0,0,0,0,0,1,
		})	//transition matrix
	B := mat64.NewDense(9,6,[]float64{
		math.Pow(dt,2)/2,0,0,0,0,0,
		0,math.Pow(dt,2)/2,0,0,0,0,
		0,0,math.Pow(dt,2)/2,0,0,0,
		dt,0,0,0,0,0,
		0,dt,0,0,0,0,
		0,0,dt,0,0,0,
		0,0,0,dt,0,0,
		0,0,0,0,dt,0,
		0,0,0,0,0,dt,
		})	//control input matrix
	H := mat64.NewDense(6,9,[]float64{
		1,0,0,0,0,0,0,0,0,
		0,1,0,0,0,0,0,0,0,
		0,0,1,0,0,0,0,0,0,
		0,0,0,1,0,0,0,0,0,
		0,0,0,0,1,0,0,0,0,
		0,0,0,0,0,1,0,0,0,
		})	//measurement matrix
	X := mat64.NewDense(9,1,[]float64{0,
                                      0,
                                      0,
                                      0,
                                      0,
                                      0,
                                      0,
                                      0,
                                      0})	//initial state vector: x/y/z pos, x/y/z vel, orientation roll/pitch/yaw
	Xhat := X //initial estimate
	U := mat64.NewDense(6,1,[]float64{0,
                                      0,
                                      0,
                                      0,
                                      0,
                                      0})	//control matrix
	
	a.Mul(B,B.T()) //9,9
	a0 := []float64{extnoise.At(0,0),extnoise.At(1,0),extnoise.At(2,0),extnoise.At(0,0),extnoise.At(1,0),extnoise.At(2,0),extnoise.At(3,0),extnoise.At(4,0),extnoise.At(5,0)}	//extnoise slice for Qu
	a1 := a0
	for n:=0;n<8;n++ {
		a1 = append(a1,a0...)
	}	//slice for Qu
	Qu := mat64.NewDense(9,9,a1) //proto Qu I
	Qu.MulElem(Qu,Qu)	//proto Qu II
	Qu.MulElem(Qu,&a)	//Qu
	P := Qu	//initial estimation covariance
	
	for t=0; t<duration; t+=dt {
		
		//Readings
		Z := mat64.NewDense(6,1,[]float64{0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0})	//measurement vector: x/y/z pos, x/y/z vel
		
		//Calculations
		//Xhat = F*Xhat + B*U
		b.Mul(F,Xhat)		//9,1
		c.Mul(B,U)		//9,1
		Xhat.Add(&b,&c)		//9,1
		//Inn = Z - H*Xhat
		d.Mul(H,Xhat)		//6,1
		Inn.Sub(Z,&d)		//6,1
		//InnE = H*P*H' + R
		e.Mul(H,P)		//6,9
		f.Mul(&e,H.T())		//6,6
		InnE.Add(&f,R)		//6,6
		//Pt = F*P*F'
		g.Mul(F,P)		//9,9
		g.Mul(&g, F.T())	//9,9
		Pt.Add(&g,Qu)		//9,9
		//K = (Pt + Qu)*H'*InnE^-1
		h.Mul(&Pt,H.T())	//9,6
		i.Inverse(&InnE)	//6,6
		K.Mul(&h,&i)		//9,6
		//Xhat = Xhat + K*Inn
		b.Mul(&K,&Inn)		//9,1
		Xhat.Add(Xhat,&b)	//9,1
		//P = (Pt + Qu) - (K*P*F' + Qu)
		g.Mul(&K,H)		//9,9
		j.Mul(&g,&Pt)		//9,9
		g.Add(&Pt,Qu)		//9,9
		P.Sub(&g,&j)		//9,9
	}
	
	//Xhat printout: x/y/z position, x/y/z velocity, roll/pitch/yaw orientation
	fmt.Printf("\n  Xhat = \n")
	for na:=0;na<9;na++{
		if P.At(na,0)<0{
		fmt.Printf(" %f\n",Xhat.At(na,0))
		}else{
		fmt.Printf("  %f\n",Xhat.At(na,0))
		}
	}
		
	//P printout
	fmt.Printf("\n  P = \n")
	for ni:=0;ni<9;ni++{
		for nj:=0;nj<9;nj++{
			if P.At(nj,ni)<0{
				fmt.Printf(" %f",P.At(nj,ni))
			}else{
				fmt.Printf("  %f",P.At(nj,ni))
			}
		}
		fmt.Printf("\n")
	}
	fmt.Printf("\n")
}