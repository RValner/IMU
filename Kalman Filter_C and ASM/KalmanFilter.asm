%include "macros.inc"

%ifdef WIN32
%define kalmanFilter_Iterate _kalmanFilter_Iterate
%endif

; Define source addresses
%define	angle 	 	dword[ebp - 4]		; *angle  	 @ ebp - 8
%define	rateBias 	dword[ebp - 8]		; *rateBias  @ ebp - 8
%define	angleRate	dword[ebp - 12]		; *angleRate @ ebp - 12
%define	dt			dword[ebp - 16]		; *dt		 @ ebp - 16

%define	P00			dword[ebp - 20]		; P00	 	 @ ebp - 20
%define	P01			dword[ebp - 24]		; P01	 	 @ ebp - 24
%define	P10			dword[ebp - 28]		; P10	 	 @ ebp - 28
%define	P11			dword[ebp - 32]		; P11	 	 @ ebp - 32

%define	K0			dword[ebp - 36]		; K0	 	 @ ebp - 36
%define	K1			dword[ebp - 40]		; K1		 @ ebp - 40

%define	innovation	dword[ebp - 44]		; *innovation@ ebp - 44
%define	S			dword[ebp - 48]		; *S		 @ ebp - 48
%define	Q_rateBias	dword[ebp - 52]		; *Q_rateBias@ ebp - 52
%define	Q_angle		dword[ebp - 56]		; *Q_angle	 @ ebp - 56
%define	u_angle		dword[ebp - 60]		; *u_angle	 @ ebp - 60

; Destination addresses
%define	angleDst	dword[ebx]			; *angle 	 @ ebp - 4  +
%define	rateBiasDst dword[ebx + 4]		; *rateBias  @ ebp - 8	+
%define	angleRateDst dword[ebx + 8]		; *angleRate @ ebp - 12 +

%define P00Dst		dword[ebx + 16]		; P00	 	 @ ebp - 20 +
%define	P01Dst		dword[ebx + 20]		; P01	 	 @ ebp - 24 +
%define	P10Dst		dword[ebx + 24]		; P10	 	 @ ebp - 28 +
%define	P11Dst		dword[ebx + 28]		; P11	 	 @ ebp - 32 +


globalfunc kalmanFilter_Iterate, filter:dword, measuredRate:dword, measuredScalar:dword
	
	prologue
	mov		eax, .measuredRate	; ADDRESS !!!!!!
	mov		ebx, .filter		; ADDRESS !!!!!!
	
	; Pushing filter variables into stack - Temporary variables
	push	dword[ebx]			; *angle 	 @ ebp - 4  +
	push	dword[ebx + 4]		; *rateBias  @ ebp - 8	+
	push	dword[ebx + 8]		; *angleRate @ ebp - 12 +
	push	dword[ebx + 12]		; *dt		 @ ebp - 16 +
	
	push	dword[ebx + 16]		; P00	 	 @ ebp - 20 +
	push	dword[ebx + 20]		; P01	 	 @ ebp - 24 +
	push	dword[ebx + 24]		; P10	 	 @ ebp - 28 +
	push	dword[ebx + 28]		; P11	 	 @ ebp - 32 +
	
	push	dword[ebx + 32]		; K0	 	 @ ebp - 36 +
	push	dword[ebx + 36]		; K1		 @ ebp - 40	+
	
	push	dword[ebx + 40]		; *innovation@ ebp - 44 +
	push	dword[ebx + 44]		; *S@		 @ ebp - 48 +
	push	dword[ebx + 48]		; *Q_rateBias@ ebp - 52 +
	push	dword[ebx + 52]		; *Q_angle	 @ ebp - 56 +
	push	dword[ebx + 56]		; *u_angle	 @ ebp - 60 +
	
	
	; // Prediction op1: angleRate = measuredRate - rateBias;
	fld		dword [eax]			; st0 = measuredRate
	fsub	rateBias			; st0 -= rateBias
	fst     angleRate			; store st0 @ filter->angleRate
	
	; // Prediction op2: angle += angleRate * dt;
	fmul	dt					; st0 *= dt
	fadd	angle				; st0 += angle 
	; -----1 element in st----- st0 = angle
	
	; // Innovation: innovation = measuredScalar - angle;
	mov		eax, .measuredScalar
	fld		dword [eax]			; st0 = measuredScalar, st1 = angle
	fsub	st0, st1			; st0 -= angle;
	fst    innovation			; store and pop st0 @ innovation
	; -----2 elements in st----- st0 = innovation, st1 = angle
	
	; // Prediction covariance matrix op1: 
	; P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
	fld		dt					; st0 = dt, st1 = innovation, st2 = angle
	fld		st0					; st0 = dt, st1 = dt, st2 = innovation, st3 = angle
	fmul	P11					; st0 *= P11
	fsub	P01					; st0 -= P01
	fsub	P10					; st0 -= P10
	fadd	Q_angle				; st0 += Q_angle
	fmul 	st0, st1			; st0 *= st1 //dt
	fadd	P00					; st0 += P00
	; -----4 elements in st----- st0 = P00pr, st1 = dt, st2 = innovation, st3 = angle
	
	; // Innovation error: S = P00 + u_angle;
	fld		u_angle				; st0 = u_angle, st1 = P00pr, st2 = dt, st3 = innovation, st4 = angle
	fadd	st0, st1			; st0 += st(1) //P00
	fst		S					; store st0 @ S
	; -----5 elements in st----- st0 = S, st1 = P00pr, st2 = dt, st3 = innovation, st4 = angle
	
	; // Kalman gain op1: K0 = P00pr / S;
	fdivr 	st0, st1			; st0 = P00pr / S = K0
	fst		K0					; store st0 @ K0
	; -----5 elements in st----- st0 = K0, st1 = P00pr, st2 = dt, st3 = innovation, st4 = angle
	
	; // Posteriori angle: angle += K0 * innovation;
	fxch	st3					; st0 = innovation, st1 = P00pr, st2 = dt, st3 = K0, st4 = angle
	fmul	st0, st3			; st0 = K0 * innovation
	fadd	st0, st4			; st0 += st4
	fstp	angle				; store and pop st0 @ angle
	; -----4 elements in st----- st0 = P00pr, st1 = dt, st2 = K0, st3 = angle
		
	; // Posteriori covariance matrix op1: P00po -= K0 * P00pr
	fmul 	st2, st0			; st2 *= P00pr
	fsub 	st0, st2			; st(0) -= K0 * P00pr
	fstp 	P00					; store and pop st0 @ P00po
	; -----3 elements in st----- st0 = dt, st1 = K0 * P00pr, st2 = angle
	
	; // Prediction covariance matrix op2: P01 -= dt * P11;
	fld 	P11					; st0 = P11, st1 = dt, st2 = K0 * P00pr, st3 = angle
	fmul 	st0, st1			; st0 *= dt
	fld		P01					; st0 = P01, st1 = P11*dt, st2 = dt, st3 = K0 * P00pr, st4 = angle
	fsub	st0, st1			; st0 -= st1
	; -----5 elements in st----- st0 = P01pr, st1 = P11*dt, st2 = dt, st3 = K0 * P00pr, st4 = angle
	
	; // Posteriori covariance matrix op2: P01 -= K0 * P01
	fld		K0					; st0 = K0, st1 = P01pr, st2 = P11*dt, st3 = dt, st4 = K0 * P00pr, st5 = angle
	fxch	st4					; DISCOVERED A MISTAKE WAY TOO LATE, SOME HACKING WAS NEEDED
	fstp	st0
	fmul 	st3, st0			; st3 *= st0
	fsub 	st0, st3			; st0 -= st3
	fstp 	P01					; store and pop st0 @ P01
	; -----4 elements in st----- st0 = P11*dt, st1 = dt, st2 = K0*P01new, st3 = angle
	
	; // Prediction covariance matrix op3: P10 -= dt * P11;
	fld 	P10					; st0 = P10, st1 = P11*dt, st2 = dt, st3 = K0*P01new, st4 = angle
	fsub	st0, st1			; st0 -= st1
	; -----5 elements in st----- st0 = P10new, st1 = P11*dt, st2 = dt, st3 = K0*P01new, st4 = angle
	
	; // Kalman gain op2: K1 = P10 / S;
	fld 	S					; st0 = S, st1 = P10new, st2 = P11*dt, st3 = dt, st4 = K0*P01new, st5 = angle
	fdivr 	st0, st1			; st0 = P10new / S = K1
	fst 	K1					; store st0 @ K1
	; -----6 elements in st----- st0 = K1, st1 = P10new, st2 = P11*dt, st3 = dt, st4 = K0*P01pr, st5 = angle
	
	; // Posteriori covariance matrix op3: P10 -= K1 * P00;
	fmul	P00					; st0 = K1 * P00
	fsubr	st0, st1			; st0 = P10
	fstp	P10					; store and pop st0 @ P10
	; -----5 elements in st----- st0 = P10new, st1 = P11*dt, st2 = dt, st3 = K0*P01pr, st4 = angle
	
	fstp 	st0
	fstp 	st0
	; -----3 elements in st----- st0 = dt, st1 = K0*P01new, st2 = angle
	
	;// Prediction covariance matrix op4: P11 += Q_rateBias * dt
	fmul	Q_rateBias			; st0 *= Q_rateBias
	fadd	P11					; st0 += P11 = P11new
	; -----3 elements in st----- st0 = P11pr, st1 = K0*P01pr, st2 = angle

	; // Posteriori covariance matrix op4: P11 -= K1 * P01
	fld		K1					; st0 = K1, st1 = P11pr, st2 = K0*P01pr, st3 = angle
	fmul	P01					; st0 *= P01po
	fsubr	st0, st1			; st0 = P11po
	fstp	P11					; store and pop st0 @ P11
	; -----3 elements in st----- st0 = P11pr, st1 = K0*P01pr, st2 = angle
	
	; *rateBias += K1 * innovation;
	fld		innovation
	fmul	K1
	fadd	rateBias
	fstp	rateBias
	; -----3 elements in st----- st0 = P11pr, st1 = K0*P01pr, st2 = angle
	
	fstp 	st0
	fstp 	st0
	fstp 	st0
	
	; Move values back into kalman filter struct
	mov		eax,	angle
	mov		angleDst, eax
	
	mov		eax,	angleRate
	mov		angleRateDst, eax
	
	mov		eax,	rateBias
	mov		rateBiasDst, eax
	
	mov		eax,	P00
	mov		P00Dst, eax
	
	mov		eax,	P01
	mov		P01Dst, eax
	
	mov		eax,	P10
	mov		P10Dst, eax
	
	mov		eax,	P11
	mov		P11Dst, eax
	

	xor		eax, eax
	
	
; C reference code

    ; // Prediction
	;+ *angleRate = measuredRate - *rateBias;
	;+ *angle += *angleRate * dt;

	; // Prediction covariance matrix
	;+ P00 += dt * (dt*P11 - P01 - P10 + Q_angle);
	;+ P01 -= dt * P11;
	;+ P10 -= dt * P11;
	;+ P11 += Q_rateBias * dt;

	; // Innovation
	;+innovation = measuredScalar - *angle;

	; // Innovation error
	;+ S = P00 + u_angle;

	; // Kalman gain
	;+ K0 = P00 / S;
	;+ K1 = P10 / S;

	; // Posteriori angle
	;+ *angle += K0 * innovation;
	;+ *rateBias += K1 * innovation;

	; // Posteriori covariance matrix
	;+ P00 -= K0 * P00;
	;+ P01 -= K0 * P01;
	;+ P10 -= K1 * P00;
	;+ P11 -= K1 * P01;
;}

	epilogue

%ifdef WIN32
	extern _ExitProcess@4
	globalfunc _aexit, exitCode:dword
		prologue
		invoke _ExitProcess@4, .exitCode
		epilogue
%else
	globalfunc aexit, exitCode:dword
		uses ebx
		prologue
		mov eax, 0x01      ; SYS_EXIT (terminate process)
		mov ebx, .exitCode ; exitcode
		int 0x80           ; syscall interrupt
		epilogue
%endif	