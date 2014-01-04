using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Extensions;
using UnityEngine;

class AutoRendezVel
{
    public static double s_thresholdAp = 0.05;
    public static double s_thresholdPe = 0.10;
    public static double s_thresholdInclination = 2.0;
    public static double s_thresholdMajorAxis = 0.96;

    public static double s_thresholdTimeToTurn = 30.0;
    public static double s_thresholdAngleError = 2.0;
    public static double s_thresholdAngleRate = 8.0;
    public static int s_minTurnUpdates = 30;

    public static double s_thresholdVelocity = 0.1;
    public static double s_minThrottle = 0.25;
    public static double s_maxThrottle = 0.85;
    public static double s_ratioThrottleDeltaV = 0.1;
    public static double s_thresholdAtmosphere = 1.25;

    public enum RendezState
    {
        Invalid = -1,
        Initialize = 0,     // Check AP/PE, setup modes etc.
        WaitForApPe = 1,    // Wait until the ship reaches certain position.
        TurnPhase = 2,      // Turn to prograde/retrograde.
        BurnEngine = 3,     // Burn the engine to specified velocity.
    }

    public enum RendezMode
    {
        Invalid = -1,
        BurnAtAp = 0,
        BurnAtPe = 1,
    }

	public enum BurnType
	{
		Invalid = -1,
 		ProgradeBurn = 0,
		RetrogradeBurn = 1,
	}


    public RendezState m_rendezState = RendezState.Invalid;
    public RendezMode m_rendezMode = RendezMode.Invalid;
	public BurnType m_burnType = BurnType.Invalid;
    public double m_recordedVelocity = 0;

    Vessel m_ownVessel = null;
    Vessel m_targetVessel = null;

    double m_velAtAp = 0;
    double m_velAtPe = 0;
    double m_velCurrent = 0;
    double m_headingError = 0;
    double m_turnRate = 0;
    int m_turnUpdates = 0;

    bool m_needZeroThrottle = false;

    public void activate()
    {
        m_rendezState = RendezState.Initialize;
    }

    public void deactivate()
    {
        FinalizeSystem();
    }

    public void setShips(Vessel ownVessel, Vessel targetVessel)
    {
        m_ownVessel = ownVessel;
        m_targetVessel = targetVessel;
    }

    private bool checkVessels()
    {
        return m_ownVessel != null && m_targetVessel != null;
    }

    public void setVelocities(double velAtAp, double velAtPe, double velCurrent)
    {
        m_velAtAp = velAtAp;
        m_velAtPe = velAtPe;
        m_velCurrent = velCurrent;
        //Debug.Log("setVelocities: " + m_velAtAp.ToString("F1") + " " + m_velAtPe.ToString("F1") + " " + m_velCurrent.ToString("F1"));
    }

    public void setHeadingError(double error)
    {
        m_headingError = error;
        //Debug.Log("setHeadingError: " + error.ToString("F1"));
    }

    public void setTurnRate(double turnRate)
    {
        m_turnRate = turnRate;
        //Debug.Log("setTurnRate: " + turnRate.ToString("F1"));
    }

    private bool checkRelativeInclination()
    {
        if (!checkVessels()) return false;
        return (Math.Abs(m_ownVessel.orbit.inclination - m_targetVessel.orbit.inclination) < s_thresholdInclination);
    }

    private RendezMode checkApPe()
    {
        if (!checkVessels()) return RendezMode.Invalid;
        if (m_targetVessel.orbit.eccentricity < 0.1)
        {
            Debug.Log("Circular orbit");
            double tgtAverageAlt = (m_targetVessel.orbit.ApA + m_targetVessel.orbit.PeA) / 2;
            if (Math.Abs(m_ownVessel.orbit.ApA - m_targetVessel.orbit.ApA) / (tgtAverageAlt + m_targetVessel.mainBody.Radius) < s_thresholdAp)
            {
                // My AP is really close to target's orbit.
                return RendezMode.BurnAtAp;
            }
            else if (Math.Abs(m_ownVessel.orbit.PeA - m_targetVessel.orbit.PeA) / (tgtAverageAlt + m_targetVessel.mainBody.Radius) < s_thresholdPe)
            {
                // My AP is not close but my PE is.
                return RendezMode.BurnAtPe;
            }
            else
            {
                return RendezMode.Invalid;
            }
        }
        else
        {
            int isMajorAxisAligned = 0;
            //Debug.Log("ecc: " + m_ownVessel.orbit.eccVec.ToString() + ", " + m_targetVessel.orbit.eccVec.ToString());
            Vector3 ownEccVec = m_ownVessel.orbit.eccVec.normalized;
            Vector3 tgtEccVec = m_targetVessel.orbit.eccVec.normalized;
            float dot = Vector3.Dot(ownEccVec, tgtEccVec);
            //Debug.Log("ecc dotted: " + dot.ToString("F1"));
            if (Math.Abs(dot) >= s_thresholdMajorAxis)
                isMajorAxisAligned = -1;
            if (dot >= s_thresholdMajorAxis)
                isMajorAxisAligned = 1;
            
            if (isMajorAxisAligned != 0)
            {
                if (Math.Abs(m_ownVessel.orbit.ApA - m_targetVessel.orbit.ApA) / (m_targetVessel.orbit.ApA + m_targetVessel.mainBody.Radius) < s_thresholdAp)
                {
                    return (isMajorAxisAligned == 1) ? RendezMode.BurnAtAp : RendezMode.BurnAtPe;
                }
                else if (Math.Abs(m_ownVessel.orbit.PeA - m_targetVessel.orbit.PeA) / (m_targetVessel.orbit.PeA + m_targetVessel.mainBody.Radius) < s_thresholdPe)
                {
                    return (isMajorAxisAligned == 1) ? RendezMode.BurnAtPe : RendezMode.BurnAtAp;
                }
                else if (Math.Abs(m_ownVessel.orbit.ApA - m_targetVessel.orbit.PeA) / (m_targetVessel.orbit.PeA + m_targetVessel.mainBody.Radius) < s_thresholdPe)
                {
                    return (isMajorAxisAligned == -1) ? RendezMode.BurnAtAp : RendezMode.BurnAtPe;
                }
                else if (Math.Abs(m_ownVessel.orbit.PeA - m_targetVessel.orbit.ApA) / (m_targetVessel.orbit.ApA + m_targetVessel.mainBody.Radius) < s_thresholdAp)
                {
                    return (isMajorAxisAligned == -1) ? RendezMode.BurnAtPe : RendezMode.BurnAtAp;
                }
                else
                {
                    return RendezMode.Invalid;
                }
            }
            else
            {
                return RendezMode.Invalid;
            }
        }
    }

    public void update(ref bool flyByWire, ref bool modeChanged)
    {
        if (!checkVessels()) return;

        switch (m_rendezState)
        {
            case RendezState.Initialize:
            {
                if (checkRelativeInclination() == false)
                {
                    FinalizeSystem();
                    flyByWire = false;
                    modeChanged = true;
                    return;
                }
                m_rendezMode = checkApPe();
                if (m_rendezMode == RendezMode.Invalid)
                {
                    FinalizeSystem();
                    flyByWire = false;
                    modeChanged = true;
                    return;
                }

                m_rendezState = RendezState.WaitForApPe;
                break;
            }
            case RendezState.WaitForApPe:
            {
                bool timeToExitHighWarp = false;
                if (m_rendezMode == RendezMode.BurnAtAp)
                {
                    m_recordedVelocity = m_velAtAp;
                    if (m_ownVessel.orbit.timeToAp < s_thresholdTimeToTurn * 3)
                        timeToExitHighWarp = true;
                }
                else if (m_rendezMode == RendezMode.BurnAtPe)
                {
                    m_recordedVelocity = m_velAtPe;
                    if (m_ownVessel.orbit.timeToPe < s_thresholdTimeToTurn * 3)
                        timeToExitHighWarp = true;
                }

                bool timeToExitWarp = false;
                if (m_rendezMode == RendezMode.BurnAtAp)
                {
                    m_recordedVelocity = m_velAtAp;
                    if (m_ownVessel.orbit.timeToAp < s_thresholdTimeToTurn * 2)
                        timeToExitWarp = true;
                }
                else if (m_rendezMode == RendezMode.BurnAtPe)
                {
                    m_recordedVelocity = m_velAtPe;
                    if (m_ownVessel.orbit.timeToPe < s_thresholdTimeToTurn * 2)
                        timeToExitWarp = true;
                }

                bool timeToTurn = false;
                if (m_rendezMode == RendezMode.BurnAtAp)
                {
                    m_recordedVelocity = m_velAtAp; 
                    if (m_ownVessel.orbit.timeToAp < s_thresholdTimeToTurn)
                        timeToTurn = true;
                }
                else if (m_rendezMode == RendezMode.BurnAtPe)
                {
                    m_recordedVelocity = m_velAtPe;
                    if (m_ownVessel.orbit.timeToPe < s_thresholdTimeToTurn)
                        timeToTurn = true;
                }

                if (timeToTurn)
                {
                    //Debug.Log("Time to turn to " + ((m_rendezMode == RendezMode.BurnAtAp) ? "Ap" : "Pe"));
                    m_rendezState = RendezState.TurnPhase;
                    m_ownVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                }
                else if (timeToExitWarp)
                {
                    TimeWarp.SetRate(0, true);
                }
                else if (timeToExitHighWarp)
                {
                    TimeWarp.SetRate(1, true);
                }

                break;
            }
            case RendezState.TurnPhase:
            {
                if ((Math.Abs(m_headingError) < s_thresholdAngleError) &&
                    (Math.Abs(m_turnRate) < s_thresholdAngleRate) && (m_turnUpdates >= s_minTurnUpdates)
                )
                {
                    //Debug.Log("Direction set, burn engine now.");
					m_rendezState = RendezState.BurnEngine;
                    m_turnUpdates = 0;
                    m_ownVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, true);
                }
                else
                {
					if (m_turnUpdates == 0)
					{
						m_burnType = (m_ownVessel.orbit.vel.magnitude > m_recordedVelocity) ? BurnType.RetrogradeBurn : BurnType.ProgradeBurn;
					}
                    m_turnUpdates++;
                }
                break;
            }
            case RendezState.BurnEngine:
            {
				if (m_burnType == BurnType.ProgradeBurn)
				{
					if (m_ownVessel.orbit.vel.magnitude - m_recordedVelocity > -s_thresholdVelocity)
					{
						//Debug.Log("Velocity reached.");
						FinalizeSystem();
						flyByWire = false;
						modeChanged = true;
						m_ownVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
					}
				}
				else if(m_burnType == BurnType.RetrogradeBurn)
				{
					if (m_ownVessel.orbit.vel.magnitude - m_recordedVelocity < s_thresholdVelocity)
					{
						//Debug.Log("Velocity reached.");
						FinalizeSystem();
						flyByWire = false;
						modeChanged = true;
						m_ownVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
					}
				}
                break;
            }
        }
    }

    private void FinalizeSystem()
    {
        m_rendezState = RendezState.Invalid;
        m_rendezMode = RendezMode.Invalid;
		m_burnType = BurnType.Invalid;
        m_ownVessel = null;
        m_targetVessel = null;
        m_recordedVelocity = 0;
        m_needZeroThrottle = true;
        m_velAtAp = 0;
        m_velAtPe = 0;
        m_velCurrent = 0;
        m_headingError = 0;
        m_turnRate = 0;
        m_turnUpdates = 0;
    }

    public void drawGUI(ref bool flyByWire, ref bool modeChanged)
    {
        if (m_rendezState == RendezState.Invalid)
        {
            bool a = GUILayout.Button("Auto Rendezvous", GUILayout.ExpandWidth(true));
            if (a)
            {
                this.activate();
                flyByWire = true;
                modeChanged = true;
            }
        }
        else
        {
            string stateStr = "";
            string burnPoint = (m_rendezMode == RendezMode.BurnAtAp) ? "Ap" : "Pe";
            string turnDir = (m_burnType == BurnType.ProgradeBurn) ? "Prograde" : "Retrograde";
            string errorStr = (m_headingError.ToString("F1") + " " + m_turnRate.ToString("F1"));
            string velStr = m_velCurrent.ToString("F1") + "/" + m_recordedVelocity.ToString("F1");
            if (m_rendezState == RendezState.Initialize) stateStr = "Initializing...";
            else if (m_rendezState == RendezState.WaitForApPe) stateStr = "Waiting for " + burnPoint + "...";
            else if (m_rendezState == RendezState.TurnPhase) stateStr = "Turning to " + turnDir + " (" + errorStr + ") ...";
            else if (m_rendezState == RendezState.BurnEngine) stateStr = "Maneuvering (" + velStr + ") ...";

            bool d = GUILayout.Button("Auto Rendezvous - " + stateStr, GUILayout.ExpandWidth(true));
            if (d)
            {
                this.deactivate();
                flyByWire = false;
                modeChanged = true;
            }
        }
    }

    public void driveShip(FlightCtrlState control)
    {
        if (m_needZeroThrottle)
        {
            control.mainThrottle = 0.0f;
            m_needZeroThrottle = false;
            return;
        }

        if (m_rendezState == RendezState.Invalid)
            return;
        
        // Here we will apply our control onto the original values.
        if (m_rendezState == RendezState.BurnEngine)
        {
            bool safeBurn = false;
            if (m_recordedVelocity > m_velCurrent) safeBurn = true;
            else if ((!m_ownVessel.mainBody.atmosphere && m_ownVessel.mainBody.Radius * s_thresholdAtmosphere < m_ownVessel.orbit.PeA ) || m_ownVessel.orbit.PeA > m_ownVessel.mainBody.maxAtmosphereAltitude * s_thresholdAtmosphere) safeBurn = true;
            if (safeBurn)
            {
                control.mainThrottle = (float)(Math.Abs(m_recordedVelocity - m_ownVessel.orbit.vel.magnitude) * s_ratioThrottleDeltaV);
                control.mainThrottle = (float)(Math.Min(s_maxThrottle, control.mainThrottle));
                control.mainThrottle = (float)(Math.Max(s_minThrottle, control.mainThrottle));
            }
            else
            {
                // Pe is falling into Atmosphere. Stop burning immediately.
                // Jump to State: Initialize to restart the procedure.
                control.mainThrottle = 0;
                m_ownVessel.ActionGroups.SetGroup(KSPActionGroup.SAS, false);
                m_rendezState = RendezState.Initialize;
            }
        }
    }
}
