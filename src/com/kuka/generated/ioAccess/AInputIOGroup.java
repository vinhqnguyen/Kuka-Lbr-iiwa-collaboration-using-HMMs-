package com.kuka.generated.ioAccess;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>AInput</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
public class AInputIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'AInput'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'AInput'
	 */
	public AInputIOGroup(Controller controller)
	{
		super(controller, "AInput");

		addInput("AInput_1", IOTypes.ANALOG, 16);
		addInput("AInput_2", IOTypes.ANALOG, 16);
		addInput("AInput_3", IOTypes.ANALOG, 16);
		addInput("AInput_4", IOTypes.ANALOG, 16);
		addInput("AInput_5", IOTypes.ANALOG, 16);
		addInput("AInput_6", IOTypes.ANALOG, 16);
		addInput("AInput_7", IOTypes.ANALOG, 16);
		addInput("AInput_8", IOTypes.ANALOG, 16);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_1'
	 */
	public double getAInput_1()
	{
		return getAnalogIOValue("AInput_1", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_2'
	 */
	public double getAInput_2()
	{
		return getAnalogIOValue("AInput_2", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_3'
	 */
	public double getAInput_3()
	{
		return getAnalogIOValue("AInput_3", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_4'
	 */
	public double getAInput_4()
	{
		return getAnalogIOValue("AInput_4", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_5'
	 */
	public double getAInput_5()
	{
		return getAnalogIOValue("AInput_5", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_6</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_6'
	 */
	public double getAInput_6()
	{
		return getAnalogIOValue("AInput_6", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_7</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_7'
	 */
	public double getAInput_7()
	{
		return getAnalogIOValue("AInput_7", false);
	}

	/**
	 * Gets the value of the <b>analog input '<i>AInput_8</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * analog input
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [0.0; 5000.0]
	 *
	 * @return current value of the analog input 'AInput_8'
	 */
	public double getAInput_8()
	{
		return getAnalogIOValue("AInput_8", false);
	}

}
