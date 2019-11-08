package frc.robot;

public class Superstructure {

    /* Handle the Singleton */
    private static Superstructure m_instance = null;

    /**
     * Get the current Superstructure instance
     * 
     * @return Current instance
     */
    public static Superstructure getInstance() {
        if (m_instance == null) {
            m_instance = new Superstructure();
        }

        return m_instance;
    }

    /* END Singleton */
    

    

}