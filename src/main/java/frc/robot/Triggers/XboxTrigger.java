package frc.robot.Triggers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxTrigger extends Trigger {
  private final XboxController m_controller;
  private final GenericHID.Hand m_hand;
  private final double m_threshold;

  public XboxTrigger(XboxController controller, GenericHID.Hand hand, double threshold) {
    m_controller = controller;
    m_hand = hand;
    m_threshold = threshold;
  }

  @Override
  public boolean get() {
    return m_controller.getTriggerAxis(m_hand) > m_threshold;
  }
}
