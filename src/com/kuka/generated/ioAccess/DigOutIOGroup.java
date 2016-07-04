package com.kuka.generated.ioAccess;

import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.ioModel.AbstractIOGroup;
import com.kuka.roboticsAPI.ioModel.IOTypes;

/**
 * Automatically generated class to abstract I/O access to I/O group <b>DigOut</b>.<br>
 * <i>Please, do not modify!</i>
 * <p>
 * <b>I/O group description:</b><br>
 * ./.
 */
public class DigOutIOGroup extends AbstractIOGroup
{
	/**
	 * Constructor to create an instance of class 'DigOut'.<br>
	 * <i>This constructor is automatically generated. Please, do not modify!</i>
	 *
	 * @param controller
	 *            the controller, which has access to the I/O group 'DigOut'
	 */
	public DigOutIOGroup(Controller controller)
	{
		super(controller, "DigOut");

		addDigitalOutput("DigOut_1", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_2", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_3", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_4", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_5", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_6", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_7", IOTypes.BOOLEAN, 1);
		addDigitalOutput("DigOut_8", IOTypes.BOOLEAN, 1);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_1'
	 */
	public boolean getDigOut_1()
	{
		return getBooleanIOValue("DigOut_1", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_1</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_1'
	 */
	public void setDigOut_1(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_1", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_2'
	 */
	public boolean getDigOut_2()
	{
		return getBooleanIOValue("DigOut_2", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_2</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_2'
	 */
	public void setDigOut_2(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_2", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_3'
	 */
	public boolean getDigOut_3()
	{
		return getBooleanIOValue("DigOut_3", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_3</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_3'
	 */
	public void setDigOut_3(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_3", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_4'
	 */
	public boolean getDigOut_4()
	{
		return getBooleanIOValue("DigOut_4", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_4</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_4'
	 */
	public void setDigOut_4(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_4", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_5'
	 */
	public boolean getDigOut_5()
	{
		return getBooleanIOValue("DigOut_5", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_5</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_5'
	 */
	public void setDigOut_5(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_5", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_6</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_6'
	 */
	public boolean getDigOut_6()
	{
		return getBooleanIOValue("DigOut_6", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_6</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_6'
	 */
	public void setDigOut_6(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_6", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_7</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_7'
	 */
	public boolean getDigOut_7()
	{
		return getBooleanIOValue("DigOut_7", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_7</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_7'
	 */
	public void setDigOut_7(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_7", value);
	}

	/**
	 * Gets the value of the <b>digital output '<i>DigOut_8</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @return current value of the digital output 'DigOut_8'
	 */
	public boolean getDigOut_8()
	{
		return getBooleanIOValue("DigOut_8", true);
	}

	/**
	 * Sets the value of the <b>digital output '<i>DigOut_8</i>'</b>.<br>
	 * <i>This method is automatically generated. Please, do not modify!</i>
	 * <p>
	 * <b>I/O direction and type:</b><br>
	 * digital output
	 * <p>
	 * <b>User description of the I/O:</b><br>
	 * ./.
	 * <p>
	 * <b>Range of the I/O value:</b><br>
	 * [false; true]
	 *
	 * @param value
	 *            the value, which has to be written to the digital output 'DigOut_8'
	 */
	public void setDigOut_8(java.lang.Boolean value)
	{
		setDigitalOutput("DigOut_8", value);
	}

}
