/* |------------------------------------------------------------------------| */
/* | Implementação Elevador (Firmware)                                      | */
/* |------------------------------------------------------------------------| */
/* | Curso: Engenharia de Computação                                        | */
/* | Disciplina: Microcontroladores e Aplicações                            | */
/* | Professor: Erick Barboza                                               | */
/* | Alunos: Bruno Lemos, Karla Sophia, Leticia Gabriela e Maria Fernanda   | */
/* |------------------------------------------------------------------------| */

/* ---------------------Definições das variáveis globais--------------------- */

#define CLOCK 16.0e6
#define DELAY_TIMER_A 1.0

#define PRESCALE_TIMER_A 0b101
#define PRESCALE_DIV_TIMER_A 1024
#define WGM_TIMER_A 0b0100

/* ---------------------Definições das variáveis globais--------------------- */
/* -------------------------------------------------------------------------- */
/* ------------------------Definição dos registradores----------------------- */

// Registrador para uso geral
.def temp = R16

// Registradores usados para armazenar o estado do elevador
.def estado = r20
.def floor = r21
.def next_floor = r22
.def direction = r23
.def counter = r24

/* ------------------------Definição dos registradores----------------------- */
/* -------------------------------------------------------------------------- */
/* -------------------------Definição dos pinos usados----------------------- */

.equ pin_button_external_floor_0 = PB5
.equ pin_button_external_floor_1 = PB4
.equ pin_button_external_floor_2 = PB3
.equ pin_button_external_floor_3 = PB2

.equ pin_button_internal_floor_0 = PB1
.equ pin_button_internal_floor_1 = PB0
.equ pin_button_internal_floor_2 = PD7
.equ pin_button_internal_floor_3 = PD6

.equ pin_button_open_door = PD5
.equ pin_button_close_door = PD4

.equ pin_buzzer = PC5
.equ pin_led = PC4

/* ------------------------Definição dos registradores----------------------- */
/* -------------------------------------------------------------------------- */
/* -----------------------Definindo a fila das chamadas---------------------- */

.dseg
queue: .BYTE 4

/* -----------------------Definindo a fila das chamadas---------------------- */
/* -------------------------------------------------------------------------- */
/* -----------------------Alterando a memória para cseg---------------------- */

.cseg
.org 0

/* -----------------------Alterando a memória para cseg---------------------- */
/* -------------------------------------------------------------------------- */
/* ------------------------Tabela de interrupção (IVT)----------------------- */

rjmp SETUP
.org OC1Aaddr
jmp OCI1A_Interrupt

/* ------------------------Tabela de interrupção (IVT)----------------------- */
/* -------------------------------------------------------------------------- */
/* --------------------------------Interrupções------------------------------ */

OCI1A_Interrupt:
	push temp
	in temp, SREG
	push temp

	// Incrementando o contador
	inc counter

	pop temp
	out SREG, temp
	pop temp

	reti

/* --------------------------------Interrupções------------------------------ */
/* -------------------------------------------------------------------------- */
/* -----------------------------------Funções-------------------------------- */

/* ---------Clear counter: Zera o contador e o registrador de contagem------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (temp)                                           */
/* Definição da função: void Clear_counter()                                  */
Clear_counter:
	push temp

	// Zerando o contador
	clr counter

	// Zerando o registrador de contagem
	clr temp
	sts TCNT1L, temp
	sts TCNT1H, temp

	pop temp

	ret

/* ----------------Add Call: Adiciona uma nova chamada na fila--------------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16, r17, r18, r19                                   */
/* Definição da função: void Add_call(floor int, location int)                */
/* Argumentos: floor (r18), location (r19)                                    */
Add_call:
	push r16
	push r17

	// Copia o valor do argumento floor (r18) para o registrador r17
	mov r17, r18

	// Aplica a função (3 - x) para converter o andar em uma posição da fila
	ldi r16, 3
	sub r16, r17

	// Inicialização o ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Desloca o ponteiro para a posição do andar na fila
	add ZL, r16
	sbci ZH, 0

	// Lê o dado armazenado na posição da fila e salva no registrador r17
	ld r17, Z

	// Salva no valor lido a chamada atual (interna: 0b10 ou externa: 0b01)
	// O operador "or" para armazenar as chamadas internas e externas
	or r17, r19

	// Salva o novo valor na posição da fila
	st Z, r17

	pop r17
	pop r16

	ret

/* -----------Clear Call: Limpa a chamada que foi executada da fila---------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16, r17                                             */
/* Definição da função: void Clear_call()                                     */
Clear_call:
	push r16

	// Copia o valor do registrador floor para o registrador r17
	mov r17, floor

	// Aplica a função (3 - x) para converter o andar em uma posição da fila
	ldi r16, 3
	sub r16, r17

	// Inicialização o ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Desloca o ponteiro para a posição do andar na fila
	add ZL, r16
	sbci ZH, 0

	// Realiza um clear no registrador r17
	clr r17

	// Salva o novo valor na posição da fila (limpa a posição da fila)
	st Z, r17

	pop r16

	ret

/* ---Get next floor: Atribui ao registrador next_floor o novo andar alvo---- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16, r17, r18, r19                                   */
/* Definição da função: void Get_next_floor()                                 */
Get_next_floor:
	push r16
	push r17
	push r18
	push r19

	// Copia o valor do registrador floor para o registrador r17
	mov r17, floor

	// Aplica a função (3 - x) para converter o andar em uma posição da fila
	ldi r16, 3
	// Salvando no registrador r16 o andar (convertido para a posição na fila)
	sub r16, r17

	// Salvando o valor 3 no registrador r17 para converter a posição de retorno em um andar
	ldi r17, 3

	// Inicialização o ponteiro Z
	ldi ZL, low(queue)
	ldi ZH, high(queue)

	// Switch case para a variável direction
	switch_direction:
		cpi direction, 0
		breq case_direction_stop
		cpi direction, 1
		breq case_direction_up
		cpi direction, -1
		breq case_direction_down

	case_direction_stop:
		// Inicializa o registrador r18 com zero (para realizar o loop de 0 a 3)
		clr r18
		while_direction_stop:
			// Verificar se já passou por todos os valores
			cpi r18, 4
			brge end_while_direction_stop

			// Lê a posição atual da fila
			ld r19, Z+ 

			// Verifica se existe uma chamada
			cpi r19, 0
			// Caso não exista ele pula o retorno
			breq skip_direction_stop

			// Convertendo a posição encontrada para um andar (3 - posição)
			sub r17, r18

			// Salvando a nova posição no registrador new_floor
			mov next_floor, r17

			rjmp found_result

			skip_direction_stop:
				inc r18
				rjmp while_direction_stop
		end_while_direction_stop:

		rjmp end_switch_direction
	case_direction_up:
		// Desloca o ponteiro para a posição do andar na fila
		add ZL, r16
		sbci ZH, 0

		// Inicializa o registrador r18 com o valor do registrador 16 (andar atual na fila)
		mov r18, r16
		while_direction_up_internal:
			// Verificar se já passou por todos os valores
			cpi r18, 0
			brlt end_while_direction_up_internal

			// Lê a posição atual da fila
			ld r19, Z

			// Decrementa o ponteiro Z
			subi ZL, 1
			sbci ZH, 0

			// Faz uma máscara no bit referente a chamada interna
			andi r19, 0b00000010
			// Verifica se existe uma chamada interna
			cpi r19, 0
			// Caso não exista ele pula o retorno
			breq skip_direction_up_internal

			// Convertendo a posição encontrada para um andar (3 - posição)
			sub r17, r18

			// Salvando a nova posição no registrador new_floor
			mov next_floor, r17

			rjmp found_result

			skip_direction_up_internal:
				dec r18
				rjmp while_direction_up_internal
		end_while_direction_up_internal:

		// Reseta a posição do ponteiro Z
		ldi ZL, low(queue)
		ldi ZH, high(queue)

		// Inicializa o registrador r18 com o valor zero
		clr r18
		// Incrementa o valor de r16 em 1
		inc r16
		while_direction_up_external:
			// Verificar se já passou por todos os valores
			cp r18, r16
			brge end_while_direction_up_external

			// Lê a posição atual da fila
			ld r19, Z+

			// Faz uma máscara no bit referente a chamada externa
			andi r19, 0b00000001
			// Verifica se existe uma chamada externa
			cpi r19, 0
			// Caso não exista ele pula o retorno
			breq skip_direction_up_external

			// Convertendo a posição encontrada para um andar (3 - posição)
			sub r17, r18

			// Salvando a nova posição no registrador new_floor
			mov next_floor, r17

			rjmp found_result

			skip_direction_up_external:
				inc r18
				rjmp while_direction_up_external
		end_while_direction_up_external:

		rjmp end_switch_direction
	case_direction_down:
		// Desloca o ponteiro para a posição do andar na fila
		add ZL, r16
		sbci ZH, 0

		// Inicializa o registrador r18 com o valor do registrador 16 (andar atual na fila)
		mov r18, r16
		while_direction_down:
			// Verificar se já passou por todos os valores
			cpi r18, 4
			brge end_while_direction_down

			// Lê a posição atual da fila
			ld r19, Z+

			// Verifica se existe uma chamada
			cpi r19, 0
			// Caso não exista ele pula o retorno
			breq skip_direction_down

			// Convertendo a posição encontrada para um andar (3 - posição)
			sub r17, r18

			// Salvando a nova posição no registrador new_floor
			mov next_floor, r17

			rjmp found_result

			skip_direction_down:
				inc r18
				rjmp while_direction_down
		end_while_direction_down:

		rjmp end_switch_direction
	end_switch_direction:
		
	ldi next_floor, -1

	found_result:

	pop r19
	pop r18
	pop r17
	pop r16

	ret

/* -------Update Display: Uma subrotina que atualiza o andar no display------ */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (temp)                                           */
Update_display:
	push temp

	// Copiando o registrador floor para o registrador temp
	mov temp, floor
	// Movendo o andar atual para a posição do pino 2 e 3 da Porta D
	lsl temp
	lsl temp

	// Escrevendo o novo valor do display na Porta D
	out PORTD, temp

	pop temp

	ret

/* -----------Delay 20ms: Uma subrotina que aciona um delay de 20ms---------- */
/* -------------------------------------------------------------------------- */
/* Registradores usados: r16 (temp), r17, r18                                 */
/* Assumindo que o clock é 16Mhz, para 20ms são necessários 320.000 ciclos    */
/* CLOCK (16.0e6) * 20ms = CLOCK (16.0e6) * 0.02 = 320.000                    */
Delay_20ms:
	push r16
	push r17
	push r18

	// Calculando a quantidade de ciclos que o loop abaixo deve ser executado
	// Como no loop cada iteração vai consumir cinco ciclos, precisamos dividir os 320.000 ciclos por cinco
	.equ required_cycles = int(CLOCK * 0.02 / 5.0)

	// Salvando os valores em três registradores
	ldi r18, byte3(required_cycles)
	ldi r17, high(required_cycles)
	ldi r16, low(required_cycles)

	// Executando os 320.000 cliclos
	subi r16, 1
	sbci r17, 0
	sbci r18, 0
	brcc pc-3

	pop r18
	pop r17
	pop r16

	ret

/* -----------------------------------Funções-------------------------------- */

SETUP:
	/* ------------------------Inicialização da pilha------------------------ */

	ldi temp, high(RAMEND)
	out SPH, temp
	ldi temp, low(RAMEND)
	out SPL, temp

	/* ------------------------Inicialização da pilha------------------------ */
	/* ---------------------------------------------------------------------- */
	/* ---------------------Definição das portas B, D e C-------------------- */

	/* Porta B                                                                */
	/*                                                                        */
	/* PB5 = Andar 0 (Botões externos) - Entrada                              */
	/* PB4 = Andar 1 (Botões externos) - Entrada                              */
	/* PB3 = Andar 2 (Botões externos) - Entrada                              */
	/* PB2 = Andar 3 (Botões externos) - Entrada                              */
	/* PB1 = Andar 0 (Botões internos) - Entrada                              */
	/* PB0 = Andar 1 (Botões internos) - Entrada                              */
	ldi temp, 0b00000000

	// Configurando a Porta B
	out DDRB, temp

	/* Porta D                                                                */
	/*                                                                        */
	/* PD7 = Andar 2 (Botões internos) - Entrada                              */
	/* PD6 = Andar 3 (Botões internos) - Entrada                              */
	/* PD5 = Abrir Porta - Entrada                                            */
	/* PD4 = Fechar Porta - Entrada                                           */
	/* PD3 = Display 7 segmentos (Entrada B) - Saída                          */
	/* PD2 = Display 7 segmentos (Entrada A) - Saída                          */
	ldi temp, 0b00001100

	// Configurando a Porta D
	out DDRD, temp

	/* Porta C                                                                */
	/*                                                                        */
	/* PC5 = Buzzer - Saída                                                   */
	/* PC4 = Led - Saída                                                      */
	ldi temp, 0b00110000

	// Configurando a Porta C
	out DDRC, temp

	/* ---------------------Definição das portas B, D e C-------------------- */
	/* ---------------------------------------------------------------------- */
	/* -------------Definindo o estado inicial dos pinos de Saída------------ */

	/* Porta C                                                                */
	/*                                                                        */
	/* PC5 = Buzzer - 0                                                       */
	/* PC4 = Led - 0                                                          */
	cbi PORTC, pin_buzzer
	cbi PORTC, pin_led

	/* -------------Definindo o estado inicial dos pinos de Saída------------ */
	/* ---------------------------------------------------------------------- */
	/* --------------Definindo o estado inicial dos registradores------------ */

	// Atribuindo os valores aos registradores
	ldi estado, 1
	ldi floor, 0
	ldi next_floor, -1
	ldi direction, 0
	ldi counter, 0

	// Atualizando o display para o andar inicial
	call Update_display

	/* --------------Definindo o estado inicial dos registradores------------ */
	/* ---------------------------------------------------------------------- */
	/* -------------------------Inicializando a fila------------------------- */

	ldi temp, 0

	sts queue, temp
	sts queue + 1, temp
	sts queue + 2, temp
	sts queue + 3, temp

	/* -------------------------Inicializando a fila------------------------- */
	/* ---------------------------------------------------------------------- */
	/* ------------------------Configuração do Timer A----------------------- */
	
	// Calculando o valor para o TOP
	.equ TOP = int(0.5 + ((CLOCK / PRESCALE_DIV_TIMER_A) * DELAY_TIMER_A))

	// Validando se ele está dentro do limite esperado (65.532)
	.if TOP > 65535
	.error "TOP is out of range"
	.endif

	// Atribuindo o valor de comparação (TOP)
	ldi temp, high(TOP)
	sts OCR1AH, temp
	ldi temp, low(TOP)
	sts OCR1AL, temp

	// Atribuindo o valor para o modo de operação (WGM)
	ldi temp, ((WGM_TIMER_A & 0b11) << WGM10)
	sts TCCR1A, temp
	ldi temp, ((WGM_TIMER_A >> 2) << WGM12) | (PRESCALE_TIMER_A << CS10)
	sts TCCR1B, temp

	// Habilita o bit na posição OCIE1A
	lds temp, TIMSK1
	sbr temp, 1 << OCIE1A
	sts TIMSK1, temp

	sei

	/* ------------------------Configuração do Timer A----------------------- */

	rjmp MAIN

MAIN:
	/* ----------------Leitura dos botões (internos e externos)-------------- */
	sbic PINB, pin_button_external_floor_0
	rjmp button_external_floor_0_pressed

	sbic PINB, pin_button_external_floor_1
	rjmp button_external_floor_1_pressed

	sbic PINB, pin_button_external_floor_2
	rjmp button_external_floor_2_pressed

	sbic PINB, pin_button_external_floor_3
	rjmp button_external_floor_3_pressed

	sbic PINB, pin_button_internal_floor_0
	rjmp button_internal_floor_0_pressed

	sbic PINB, pin_button_internal_floor_1
	rjmp button_internal_floor_1_pressed

	sbic PIND, pin_button_internal_floor_2
	rjmp button_internal_floor_2_pressed

	sbic PIND, pin_button_internal_floor_3
	rjmp button_internal_floor_3_pressed
	/* ----------------Leitura dos botões (internos e externos)-------------- */

	rjmp switch_state

	/* -------Preparando os atributos para executar a função (Add_call)------ */
	button_external_floor_0_pressed:
		ldi r18, 0
		ldi r19, 1
		
		rjmp button_pressed

	button_external_floor_1_pressed:
		ldi r18, 1
		ldi r19, 1
		
		rjmp button_pressed

	button_external_floor_2_pressed:
		ldi r18, 2
		ldi r19, 1
		
		rjmp button_pressed

	button_external_floor_3_pressed:
		ldi r18, 3
		ldi r19, 1
		
		rjmp button_pressed

	button_internal_floor_0_pressed:
		ldi r18, 0
		ldi r19, 2
		
		rjmp button_pressed

	button_internal_floor_1_pressed:
		ldi r18, 1
		ldi r19, 2
		
		rjmp button_pressed

	button_internal_floor_2_pressed:
		ldi r18, 2
		ldi r19, 2
		
		rjmp button_pressed

	button_internal_floor_3_pressed:
		ldi r18, 3
		ldi r19, 2
		
		rjmp button_pressed

	button_pressed:
		call Add_call
		call Delay_20ms
	/* -------Preparando os atributos para executar a função (Add_call)------ */
	/* ---------------------------------------------------------------------- */
	/* -------------------------Início switch(estado)------------------------ */
	switch_state:
		cpi estado, 1
		breq case_idle
		cpi estado, 2
		breq case_elevator_ups
		cpi estado, 3
		breq case_elevator_downs
		cpi estado, 4
		breq case_new_floor
		cpi estado, 5
		breq case_elevator_stops
		cpi estado, 6
		breq case_buzzer_warning
		cpi estado, 7
		breq case_holding_door
		cpi estado, 8
		breq case_next_destiny

		case_idle:
			rjmp case_idle_longjump
		case_elevator_ups:
			rjmp case_elevator_ups_longjump
		case_elevator_downs:
			rjmp case_elevator_downs_longjump
		case_new_floor:
			rjmp case_new_floor_longjump
		case_elevator_stops:
			rjmp case_elevator_stops_longjump
		case_buzzer_warning:
			rjmp case_buzzer_warning_longjump
		case_holding_door:
			rjmp case_holding_door_longjump
		case_next_destiny:
			rjmp case_next_destiny_longjump

		/* -------------------------------Cases------------------------------ */
		case_idle_longjump:
			// Verificando se o botão de abrir a porta está sendo pressionado
			sbic PIND, pin_button_open_door
			rjmp call_in_current_floor

			call Get_next_floor

			// Se não tiver chamada, ir para "no_call"
			cpi next_floor, -1
			breq no_call

			// Se a chamada for no andar atual, ir para "call_in_current_floor"
			cp next_floor, floor
			breq call_in_current_floor

			// Se a chamada for para subir, ir para "call_up"
			cp next_floor, floor
			brge call_up

			call_down:
				ldi direction, -1
				// Mudando o estado para "case_elevator_downs"
				ldi estado, 3
				// Resetando o contador
				call Clear_counter

				rjmp end_switch_state

			call_up:
				ldi direction, 1
				// Mudando o estado para "case_elevator_ups"
				ldi estado, 2
				// Resetando o contador
				call Clear_counter

				rjmp end_switch_state

			call_in_current_floor:
				ldi direction, 0
				// Mudando o estado para "case_elevator_stops"
				ldi estado, 5
				// Resetando o contador
				call Clear_counter

				rjmp end_switch_state

			no_call:
				ldi direction, 0

			rjmp end_switch_state
		case_elevator_ups_longjump:
			// Verificando se já se passaram 3s
			cpi counter, 3
			brne wait_elevator_ups_done

			// Incrementando o andar atual
			inc floor
			// Atualizando o valor do display
			call Update_display

			// Mudando o estado para "case_new_floor"
			ldi estado, 4
			// Resetando o contador
			call Clear_counter

			wait_elevator_ups_done:

			rjmp end_switch_state
		case_elevator_downs_longjump:
			// Verificando se já se passaram 3s
			cpi counter, 3
			brne wait_elevator_downs_done

			// Decrementando o andar atual
			dec floor
			// Atualizando o valor do display
			call Update_display

			// Mudando o estado para "case_new_floor"
			ldi estado, 4
			// Resetando o contador
			call Clear_counter

			wait_elevator_downs_done:

			rjmp end_switch_state
		case_new_floor_longjump:
			// Comparando se o andar atual é o andar alvo
			cp floor, next_floor
			breq stop_elevator

			// Caso não seja o andar alvo, voltar para "case_idle"
			ldi estado, 1

			rjmp end_switch_state

			stop_elevator:
				// Caso seja o andar alvo, ir para o próximo estado "case_elevator_stops"
				ldi estado, 5

			rjmp end_switch_state
		case_elevator_stops_longjump:
			// Verificando se o botão de fechar a porta foi pressionado
			sbis PIND, pin_button_close_door
			rjmp skip_close_door_a

			// Mudando o estado para "case_next_destiny"
			ldi estado, 8
			// Resetando o contador
			call Clear_counter

			rjmp end_switch_state

			skip_close_door_a:
				// Ativando o led
				sbi PORTC, pin_led

				// Verificando se já se passaram 5s
				cpi counter, 5
				brne end_switch_state

				// Mudando o estado para "case_buzzer_warning"
				ldi estado, 6
				// Resetando o contador
				call Clear_counter

			rjmp end_switch_state
		case_buzzer_warning_longjump:
			// Verificando se o botão de fechar a porta foi pressionado
			sbis PIND, pin_button_close_door
			rjmp skip_close_door_b

			// Mudando o estado para "case_next_destiny"
			ldi estado, 8
			// Resetando o contador
			call Clear_counter

			rjmp end_switch_state

			skip_close_door_b:
				// Ativando o buzzer
				sbi PORTC, pin_buzzer

				// Verificando se já se passaram 5s
				cpi counter, 5
				brne end_switch_state

				// Mudando o estado para "case_holding_door"
				ldi estado, 7
				// Resetando o contador
				call Clear_counter

			rjmp end_switch_state
		case_holding_door_longjump:
			// Verificando se o botão de abrir a porta está sendo pressionado
			sbis PIND, pin_button_open_door
			rjmp close_door
		
			// Aplicando um delay para evitar o bounce
			call Delay_20ms

			rjmp end_switch_state

			close_door:
				// Mudando estado para "case_next_destiny"
				ldi estado, 8

			rjmp end_switch_state
		case_next_destiny_longjump:
			// Desativando o buzzer e o led
			cbi PORTC, pin_buzzer
			cbi PORTC, pin_led

			// Removendo a chamada da fila
			call Clear_call

			// Mudando estado para "case_idle"
			ldi estado, 1

			rjmp end_switch_state
		end_switch_state:
		/* -------------------------------Cases------------------------------ */
	rjmp MAIN