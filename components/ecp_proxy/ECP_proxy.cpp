/*
 * ECP_proxy.cpp
 *
 *  Created on: Dec 20, 2009
 *      Author: konrad
 */

#include <ocl/ComponentLoader.hpp>

#include <messip_dataport.h>

#include "ECP_proxy.h"

namespace orocos_test
{

using namespace mrrocpp;

ECP_proxy::ECP_proxy(std::string name) :
	TaskContext(name), Joint_Setpoint_port("Joint_Setpoint_output"),
			Joint_Position_port("Joint_Position_input"),
			Cartesian_Setpoint_port("Cartesian_Setpoint_output"),
			Cartesian_Position_port("Cartesian_Position_input")
{
	this->ports()->addPort(&Joint_Setpoint_port);
	this->ports()->addPort(&Joint_Position_port);

	this->ports()->addPort(&Cartesian_Setpoint_port);
	this->ports()->addPort(&Cartesian_Position_port);

	number_of_servos = 7;

}

ECP_proxy::~ECP_proxy()
{

}

bool ECP_proxy::configureHook()
{
	Joint_Setpoint.resize(number_of_servos, 0.0);
	Joint_Position.resize(number_of_servos, 0.0);

	if (hasPeer("transformator"))
	{
		RTT::TaskContext *trans = getPeer("transformator");
		toolFrame_ext_prop = trans->properties()->getProperty<KDL::Frame> (
				"ToolFrame");
		if (!toolFrame_ext_prop.ready())
		{
			log(RTT::Error) << "ToolFrame not implemented in transformator"
					<< RTT::endlog();
			return false;
		}
	}
	else
	{
		log(RTT::Error) << "transformator not found" << RTT::endlog();
		return false;
	}
	return true;
}

bool ECP_proxy::startHook()
{
	std::string server_attach_point("irp6_on_track");
	attach = messip::port_create(server_attach_point);

	if (attach == NULL)
	{
		log(RTT::Error) << "failend to attach" << RTT::endlog();
		return false;
	}

	state = 0;
	next_state = GET_STATE;
	real_reply_type = lib::ACKNOWLEDGE;

	return true;
}

void ECP_proxy::updateHook()
{
	// by Y pierwsza petla while do odpytania o stan EDP przez UI zaraz po starcie EDP

	if (state == 0) //while ((next_state != GET_INSTRUCTION) && (next_state != GET_SYNCHRO))
	{
		switch (next_state)
		{
		case GET_STATE:
			// wstepna interpretacja nadeslanego polecenia w celu wykrycia nieprawidlowosci
			switch (receive_instruction())
			{
			case lib::GET:
				// potwierdzenie przyjecia polecenia (dla ECP)
				//            printf("SET_GET\n");
				insert_reply_type(lib::ACKNOWLEDGE);
				reply_to_instruction();

				if ((rep_type(new_instruction)) == lib::CONTROLLER_STATE)
				{

					interpret_instruction(new_instruction);
					log(RTT::Info) << "EDP : GET CONTROLLER_STATE"
							<< RTT::endlog();
				}
				else
				{
					//throw manip_and_conv_effector::NonFatal_error_1 (INVALID_INSTRUCTION_TYPE);
				}

				break;
			case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
				// okreslenie numeru bledu
				//throw manip_and_conv_effector::NonFatal_error_1 (QUERY_NOT_EXPECTED);
			default: // blad: nieznana instrukcja
				// okreslenie numeru bledu
				//throw manip_and_conv_effector::NonFatal_error_1 (INVALID_INSTRUCTION_TYPE);
				break;
			}
			next_state = WAIT;
			break;
		case WAIT:
			if (receive_instruction() == lib::QUERY)
			{ // instrukcja wlasciwa =>
				// zle jej wykonanie, czyli wyslij odpowiedz
				log(RTT::Info) << "EDP : QUERY" << RTT::endlog();
				reply_to_instruction();
			}
			else
			{ // blad: powinna byla nadejsc instrukcja QUERY
				//throw manip_and_conv_effector::NonFatal_error_3 ( QUERY_EXPECTED );
				break;
			}

			if (0) // jesli ma zostac przeprowadzona synchronizacja
			{
				next_state = GET_SYNCHRO;
				++state;
			}
			else // jesli robot jest juz zsynchronizowany
			{
				next_state = GET_INSTRUCTION;
				++state;
				log(RTT::Info) << "Robot is initially synchronised"
						<< RTT::endlog();
			}

			break;
		default:
			break;
		}

	} // end while


	if ((state == 1) && (next_state != GET_INSTRUCTION)) //while (next_state != GET_INSTRUCTION)
	{

		switch (next_state)
		{
		case GET_SYNCHRO:
			/* Oczekiwanie na zlecenie synchronizacji robota */
			switch (receive_instruction())
			{
			case lib::SYNCHRO:

				insert_reply_type(lib::ACKNOWLEDGE);
				reply_to_instruction();

				next_state = SYNCHRO_TERMINATED;
				break;
			case lib::SET:
				// instrukcja wlasciwa => zle jej wykonanie
				/*	if (pre_synchro_motion(new_instruction))
				 {
				 // Potwierdzenie przyjecia instrukcji ruchow presynchronizacyjnych do wykonania
				 insert_reply_type(lib::ACKNOWLEDGE);
				 reply_to_instruction();
				 // Zlecenie wykonania ruchow presynchronizacyjnych
				 //interpret_instruction(new_instruction);
				 // Jezeli wystapil blad w trakcie realizacji ruchow presynchronizacyjnych,
				 // to zostanie zgloszony wyjatek:

				 // Oczekiwanie na poprawne zakoczenie synchronizacji
				 next_state = WAIT_Q;
				 }
				 else
				 // blad: jedyna instrukcja w tym stanie moze by polecenie
				 // synchronizacji lub ruchow presynchronizacyjnych
				 // Bez synchronizacji adna inna instrukcja nie moze by wykonana przez EDP
				 // Informacja o bedzie polegajcym na braku polecenia synchronizacji
				 //throw manip_and_conv_effector::NonFatal_error_1(NOT_YET_SYNCHRONISED);*/
				break;
			default: // blad: jedyna instrukcja w tym stanie moze by polecenie
				// synchronizacji lub ruchow presynchronizacyjnych
				// Bez synchronizacji adna inna instrukcja nie moze by wykonana przez EDP
				/* Informacja o bedzie polegajcym na braku polecenia synchronizacji */
				//throw manip_and_conv_effector::NonFatal_error_1(INVALID_INSTRUCTION_TYPE);
				break;
			}
			break;
		case SYNCHRO_TERMINATED:
			/* Oczekiwanie na zapytanie od ECP o status zakonczenia synchronizacji (QUERY) */
			if (receive_instruction() == lib::QUERY)
			{ // instrukcja wlasciwa => zle jej wykonanie
				// Budowa adekwatnej odpowiedzi
				insert_reply_type(lib::SYNCHRO_OK);
				reply_to_instruction();
				next_state = GET_INSTRUCTION;
				//if (msg->message("Robot is synchronised"))
				//  printf(" Nie znaleziono SR\n");
			}
			else
			{ // blad: powinna byla nadejsc instrukcja QUERY
				//throw manip_and_conv_effector::NonFatal_error_4(QUERY_EXPECTED);
			}
			break;
		case WAIT_Q:
			/* Oczekiwanie na zapytanie od ECP o status zakonczenia synchronizacji (QUERY) */
			if (receive_instruction() == lib::QUERY)
			{ // instrukcja wlasciwa => zle jej wykonanie
				// Budowa adekwatnej odpowiedzi
				reply_to_instruction();
				next_state = GET_SYNCHRO;
			}
			else
			{ // blad: powinna byla nadejsc instrukcja QUERY
				//throw manip_and_conv_effector::NonFatal_error_3(QUERY_EXPECTED);
			}
			break;
		default:
			break;
		}

	}
	else if (state == 1)
	{
		++state;
	}

	/* Nieskoczona petla wykonujca przejscia w grafie automatu (procesu EDP_MASTER) */
	if (state == 2)
	{
		switch (next_state)
		{
		case GET_INSTRUCTION:
			// wstepna interpretacja nadesanego polecenia w celu wykrycia nieprawidlowosci
			switch (receive_instruction())
			{
			case lib::SET:
			case lib::GET:
			case lib::SET_GET:
				// potwierdzenie przyjecia polecenia (dla ECP)
				// printf("SET_GET\n");
				insert_reply_type(lib::ACKNOWLEDGE);
				reply_to_instruction();
				break;
			case lib::SYNCHRO: // blad: robot jest juz zsynchronizowany
				// okreslenie numeru bledu
				//throw manip_and_conv_effector::NonFatal_error_1(ALREADY_SYNCHRONISED);
			case lib::QUERY: // blad: nie ma o co pytac - zadne polecenie uprzednio nie zostalo wydane
				// okreslenie numeru bledu
				//throw manip_and_conv_effector::NonFatal_error_1(QUERY_NOT_EXPECTED);
			default: // blad: nieznana instrukcja
				// okreslenie numeru bledu
				//throw manip_and_conv_effector::NonFatal_error_1(UNKNOWN_INSTRUCTION);
				break;
			}
			next_state = EXECUTE_INSTRUCTION;
			break;
		case EXECUTE_INSTRUCTION:
			// wykonanie instrukcji - wszelkie bledy powoduja zgloszenie wyjtku NonFatal_error_2 lub Fatal_error
			interpret_instruction(new_instruction);
			next_state = WAIT;
			break;
		case WAIT:
			if (receive_instruction() == lib::QUERY)
			{
				reply_to_instruction();
			}
			else
			{ // blad: powinna byla nadejsc instrukcja QUERY

			}
			next_state = GET_INSTRUCTION;
			break;
		default:
			break;
		}
	}

}

void ECP_proxy::stopHook()
{

}

void ECP_proxy::cleanupHook()
{

}

mrrocpp::lib::INSTRUCTION_TYPE ECP_proxy::receive_instruction(void)
{
	// oczekuje na polecenie od ECP, wczytuje je oraz zwraca jego typ
	int rcvid;
	/* Oczekiwanie na polecenie od ECP */
	/* Do your MsgReceive's here now with the chid */
	while (1)
	{

		int32_t type, subtype;
		rcvid = messip::port_receive(attach, type, subtype, new_ecp_command);

		if (rcvid == -1)
		{/* Error condition, exit */
			log(RTT::Error) << "EDP : messip receive error" << RTT::endlog();
			break;
		}
		else if (rcvid < -1)
		{
			log(RTT::Error) << "EDP : messip disconnect" << RTT::endlog();
			continue;
		}

		/* A message (presumable ours) received, handle */
		break;
	}

	log(RTT::Info) << "EDP : messip message recieved" << RTT::endlog();

	caller = rcvid;

	new_instruction = new_ecp_command.instruction;

	return new_instruction.instruction_type;
}

void ECP_proxy::reply_to_instruction(void)
{
	// Wyslanie potwierdzenia przyjecia polecenia do wykonania,
	// adekwatnej odpowiedzi na zapytanie lub
	// informacji o tym, ze przyslane polecenie nie moze byc przyjte
	// do wykonania w aktualnym stanie EDP
	// int reply_size;     // liczba bajtw wysyanej odpowiedzi

	if (!((reply.reply_type == lib::ERROR) || (reply.reply_type
			== lib::SYNCHRO_OK)))
		reply.reply_type = real_reply_type;

	if (messip::port_reply(attach, caller, 0, reply) == -1)
	{
		log(RTT::Error) << "EDP : reply error" << RTT::endlog();
	}
	real_reply_type = lib::ACKNOWLEDGE;
}

void ECP_proxy::insert_reply_type(lib::REPLY_TYPE rt)
{
	reply.reply_type = rt;
}

lib::REPLY_TYPE ECP_proxy::rep_type(const lib::c_buffer &instruction)
{

	reply.reply_type = lib::ACKNOWLEDGE;
	if (instruction.get_type & OUTPUTS_DV)
	{
		reply.reply_type = lib::INPUTS;
	}
	if (instruction.get_type & RMODEL_DV)
	{
		if (reply.reply_type == lib::ACKNOWLEDGE)
			reply.reply_type = lib::RMODEL;
		else
			reply.reply_type = lib::RMODEL_INPUTS;
	}
	if (instruction.get_type & ARM_DV)
	{
		switch (reply.reply_type)
		{
		case lib::ACKNOWLEDGE:
			reply.reply_type = lib::ARM;
			break;
		case lib::INPUTS:
			reply.reply_type = lib::ARM_INPUTS;
			break;
		case lib::RMODEL:
			reply.reply_type = lib::ARM_RMODEL;
			break;
		case lib::RMODEL_INPUTS:
			reply.reply_type = lib::ARM_RMODEL_INPUTS;
			break;
		default:
			break;
		}
	}
	//real_reply_type = reply.reply_type;
	if (instruction.set_type & ARM_DV)
	{// by Y ORIGINAL
		// if (is_set_arm()||is_set_force()) {// by Y DEBUG
		switch (reply.reply_type)
		{
		case lib::ACKNOWLEDGE:
			reply.reply_type = lib::ARM;
			break;
		case lib::INPUTS:
			reply.reply_type = lib::ARM_INPUTS;
			break;
		case lib::RMODEL:
			reply.reply_type = lib::ARM_RMODEL;
			break;
		case lib::RMODEL_INPUTS:
			reply.reply_type = lib::ARM_RMODEL_INPUTS;
			break;
		default:
			break;
		}
	}
	// by Y
	if (instruction.get_type & CONTROLLER_STATE_DV)
	{
		reply.reply_type = lib::CONTROLLER_STATE;
	}

	return reply.reply_type;
}

void ECP_proxy::interpret_instruction(lib::c_buffer &instruction)
{

	// interpretuje otrzymana z ECP instrukcje;
	// wypelnaia struktury danych TRANSFORMATORa;
	// przygotowuje odpowiedz dla ECP
	// wstepne przygotowanie bufora odpowiedzi
	rep_type(instruction); // okreslenie typu odpowiedzi
	reply.error_no.error0 = OK;
	reply.error_no.error1 = OK;
	// Wykonanie instrukcji
	switch (instruction.instruction_type)
	{
	case lib::SET:
		// tu wykonanie instrukcji SET

		if (instruction.set_type & OUTPUTS_DV) // ustawienie wyjsc
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();

		if (instruction.set_type & RMODEL_DV) // zmiana modelu robota
			setRModel(instruction);
		//log(RTT::Warning) << "not implemented yet" << RTT::endlog();

		if (instruction.set_type & ARM_DV)
		{
			// przemieszczenie koncowki
			move_arm(instruction);
			instruction.get_arm_type = instruction.set_arm_type;
			get_arm_position(instruction);
			instruction.get_arm_type = lib::INVALID_END_EFFECTOR;
		}

		break;
	case lib::GET:
		// tu wykonanie instrukcji GET
		// ustalenie formatu odpowiedzi
		switch (rep_type(instruction))
		{
		case lib::CONTROLLER_STATE:
			// odczytanie TCP i orientacji koncowki
			reply.controller_state.is_power_on = true;
			reply.controller_state.is_robot_blocked = false;
			reply.controller_state.is_synchronised = true;
			reply.controller_state.is_wardrobe_on = false;

			break;
		case lib::ARM:
			// odczytanie TCP i orientacji koncowki
			get_arm_position(instruction);
			break;
		case lib::RMODEL:
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			getRModel(instruction);
			break;
		case lib::INPUTS:
			// odczytanie wejsc
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::ARM_RMODEL:
			// odczytanie TCP i orientacji koncowki
			// get_arm_position(true);
			//get_arm_position(instruction);
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::ARM_INPUTS:
			// odczytanie wej
			//get_inputs(&reply);
			// odczytanie TCP i orientacji koncowki
			// get_arm_position(true);
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::RMODEL_INPUTS:
			// ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
			if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
				;
			// get_algorithms();
			//master_order(MT_GET_ALGORITHMS, 0);
			// odczytanie wej
			//get_inputs(&reply);
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::ARM_RMODEL_INPUTS:
			// odczytanie wejsc
			//get_inputs(&reply);
			// odczytanie TCP i orientacji koncowki
			// get_arm_position(true);
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			//get_rmodel(instruction);
			break;
		default: // blad
			// ustawi numer bledu
			//throw NonFatal_error_2(INVALID_REPLY_TYPE);
			break;
		}
		break;
	case lib::SET_GET:
		// tu wykonanie instrukcji SET i GET
		// Cz SET
		if (instruction.set_type & OUTPUTS_DV)
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
		if (instruction.set_type & RMODEL_DV)
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
		if (instruction.set_type & ARM_DV)
			move_arm(instruction);

		switch (rep_type(instruction))
		{
		case lib::CONTROLLER_STATE:
			// odczytanie TCP i orientacji koncowki
			// get_arm_position(true);
			//master_order(MT_GET_CONTROLLER_STATE, 0);
			reply.controller_state.is_power_on = true;
			reply.controller_state.is_robot_blocked = false;
			reply.controller_state.is_synchronised = true;
			reply.controller_state.is_wardrobe_on = false;
			break;
		case lib::ARM:
			get_arm_position(instruction);
			break;
		case lib::RMODEL:
			//if (!(instruction.set_type & ARM_DV))
			// ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
			//if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
			// get_algorithms();
			//master_order(MT_GET_ALGORITHMS, 0);
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			getRModel(instruction);
			break;
		case lib::INPUTS:
			// odczytanie wej
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::ARM_RMODEL:
			// odczytanie TCP i orientacji koncowki
			//if (instruction.set_type & ARM_DV)
			;//get_arm_position(false, instruction);
			//else
			// get_arm_position(true);
			//get_arm_position(instruction);
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::ARM_INPUTS:
			// odczytanie wejsc
			//get_inputs(&reply);
			// odczytanie TCP i orientacji koncowki
			//if (instruction.set_type & ARM_DV)
			;//get_arm_position(false, instruction);
			//else
			// get_arm_position(true);
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::RMODEL_INPUTS:
			if (!(instruction.set_type & ARM_DV))
				// ewentualna aktualizacja numerow algorytmow i ich zestawow parametrow
				if (instruction.get_rmodel_type == lib::SERVO_ALGORITHM)
					//   get_algorithms();
					;//master_order(MT_GET_ALGORITHMS, 0);
			// odczytanie wej
			//get_inputs(&reply);
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			getRModel(instruction);
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		case lib::ARM_RMODEL_INPUTS:
			// odczytanie wejsc
			//get_inputs(&reply);
			//if (instruction.set_type & ARM_DV)
			//	;//get_arm_position(false, instruction);
			//else
			// get_arm_position(true);
			//get_arm_position(instruction);
			// odczytanie aktualnie uzywanego modelu robota (narzedzie, model kinematyczny,
			// jego korektor, nr algorytmu regulacji i zestawu jego parametrow)
			getRModel(instruction);
			log(RTT::Warning) << "not implemented yet" << RTT::endlog();
			break;
		default: // blad
			break;
		}
		break;
	default: // blad
		break;
	}

}

void ECP_proxy::get_arm_position(lib::c_buffer &instruction)
{
	switch (instruction.get_arm_type)
	{

	case lib::JOINT:
		// przeliczenie wspolrzednych do poziomu, ktory ma byc odczytany
		reply.arm_type = lib::JOINT;
		Joint_Position_port.Get(Joint_Position);
		for (int i = 0; i < 7; i++)
			reply.arm.pf_def.arm_coordinates[i] = Joint_Position[i];
		reply.arm.pf_def.gripper_coordinate = 1.2223;
		break;
	case lib::FRAME:
		Cartesian_Position_port.Get(Cartesian_Position);
		reply.arm_type = lib::FRAME;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				reply.arm.pf_def.arm_frame[i][j] = Cartesian_Position(i, j);
		break;
	case lib::MOTOR:
	default:
		log(RTT::Error) << "EDP : not implemented yet" << RTT::endlog();
	}

}

void ECP_proxy::move_arm(lib::c_buffer &instruction)
{

	switch (instruction.set_arm_type)
	{

	case lib::JOINT:
		switch (instruction.motion_type)
		{
		case lib::ABSOLUTE: // ruch bezwzgledny
			for (int i = 0; i < number_of_servos; i++)
				Joint_Setpoint[i] = instruction.arm.pf_def.arm_coordinates[i];
			break;
		case lib::RELATIVE: // ruch wzgledny
			for (int i = 0; i < number_of_servos; i++)
				Joint_Setpoint[i] += instruction.arm.pf_def.arm_coordinates[i];
			break;
		default:
			break;
		}
		Joint_Setpoint_port.Set(Joint_Setpoint);
		break;
	case lib::FRAME:
		switch (instruction.motion_type)
		{
		case lib::ABSOLUTE: // ruch bezwzgledny
			log(RTT::Error) << "EDP : Frame absolute move" << RTT::endlog();
			Cartesian_Setpoint = KDL::Frame(KDL::Rotation(
					instruction.arm.pf_def.arm_frame[0][0],
					instruction.arm.pf_def.arm_frame[0][1],
					instruction.arm.pf_def.arm_frame[0][2],
					instruction.arm.pf_def.arm_frame[1][0],
					instruction.arm.pf_def.arm_frame[1][1],
					instruction.arm.pf_def.arm_frame[1][2],
					instruction.arm.pf_def.arm_frame[2][0],
					instruction.arm.pf_def.arm_frame[2][1],
					instruction.arm.pf_def.arm_frame[2][2]), KDL::Vector(
					instruction.arm.pf_def.arm_frame[0][3],
					instruction.arm.pf_def.arm_frame[1][3],
					instruction.arm.pf_def.arm_frame[2][3]));
			break;
		case lib::RELATIVE: // ruch wzgledny
		{
			log(RTT::Error) << "EDP : Frame relative move" << RTT::endlog();

			KDL::Frame relFrame(KDL::Rotation(reply.arm.pf_def.arm_frame[0][0],
					instruction.arm.pf_def.arm_frame[0][1],
					instruction.arm.pf_def.arm_frame[0][2],
					instruction.arm.pf_def.arm_frame[1][0],
					instruction.arm.pf_def.arm_frame[1][1],
					instruction.arm.pf_def.arm_frame[1][2],
					instruction.arm.pf_def.arm_frame[2][0],
					instruction.arm.pf_def.arm_frame[2][1],
					instruction.arm.pf_def.arm_frame[2][2]), KDL::Vector(
					instruction.arm.pf_def.arm_frame[0][3],
					instruction.arm.pf_def.arm_frame[1][3],
					instruction.arm.pf_def.arm_frame[2][3]));
			Cartesian_Setpoint = Cartesian_Setpoint * relFrame;
		}
			break;
		default:
			break;
		}
		log(RTT::Error) << "EDP : Frame data send" << RTT::endlog();
		Cartesian_Setpoint_port.Set(Cartesian_Setpoint);
		log(RTT::Error) << "cart pos recieved :"
				<< instruction.arm.pf_def.arm_frame[0][3]
				<< instruction.arm.pf_def.arm_frame[1][3]
				<< instruction.arm.pf_def.arm_frame[2][3] << RTT::endlog();
		break;
	case lib::MOTOR:
	default: // blad: niezdefiniowany sposb specyfikacji pozycji koncowki
		log(RTT::Warning) << "not implemented yet" << RTT::endlog();
	}

}

void ECP_proxy::setRModel(lib::c_buffer &instruction)
{
	switch (instruction.set_rmodel_type)
	{
	case lib::TOOL_FRAME:
	{

		KDL::Frame relFrame(KDL::Rotation(
				instruction.rmodel.tool_frame_def.tool_frame[0][0],
				instruction.rmodel.tool_frame_def.tool_frame[0][1],
				instruction.rmodel.tool_frame_def.tool_frame[0][2],
				instruction.rmodel.tool_frame_def.tool_frame[1][0],
				instruction.rmodel.tool_frame_def.tool_frame[1][1],
				instruction.rmodel.tool_frame_def.tool_frame[1][2],
				instruction.rmodel.tool_frame_def.tool_frame[2][0],
				instruction.rmodel.tool_frame_def.tool_frame[2][1],
				instruction.rmodel.tool_frame_def.tool_frame[2][2]),
				KDL::Vector(instruction.rmodel.tool_frame_def.tool_frame[0][3],
						instruction.rmodel.tool_frame_def.tool_frame[1][3],
						instruction.rmodel.tool_frame_def.tool_frame[2][3]));
		toolFrame_ext_prop.set(relFrame);

	}
		break;
	case lib::ARM_KINEMATIC_MODEL:
	case lib::SERVO_ALGORITHM:
	case lib::FORCE_TOOL:
	case lib::FORCE_BIAS:
	default:
		log(RTT::Warning) << "not implemented yet" << RTT::endlog();
		break;
	}
}

void ECP_proxy::getRModel(lib::c_buffer &instruction)
{
	switch (instruction.get_rmodel_type)
	{
	case lib::TOOL_FRAME:
	{
		KDL::Frame toolFrame = toolFrame_ext_prop.get();
		reply.rmodel_type = lib::TOOL_FRAME;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 4; j++)
				reply.rmodel.tool_frame_def.tool_frame[i][j] = toolFrame(i, j);
	}
		break;
	case lib::ARM_KINEMATIC_MODEL:
	case lib::SERVO_ALGORITHM:
	case lib::FORCE_TOOL:
	case lib::FORCE_BIAS:
	default:
		log(RTT::Warning) << "not implemented yet" << RTT::endlog();
		break;
	}
}

}

ORO_CREATE_COMPONENT( orocos_test::ECP_proxy )
;
