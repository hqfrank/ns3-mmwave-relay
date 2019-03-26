 /* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
 /*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
 *  
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *  
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *  
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *  
 *   Author: Marco Miozzo <marco.miozzo@cttc.es>
 *           Nicola Baldo  <nbaldo@cttc.es>
 *  
 *   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
 *        	 	  Sourjya Dutta <sdutta@nyu.edu>
 *        	 	  Russell Ford <russell.ford@nyu.edu>
 *        		  Menglei Zhang <menglei@nyu.edu>
 */



#ifndef MMWAVE_INTERFERENCE_H
#define MMWAVE_INTERFERENCE_H

#include <ns3/object.h>
#include <ns3/packet.h>
#include <ns3/nstime.h>
#include <ns3/spectrum-value.h>
#include <string.h>
#include <ns3/mmwave-chunk-processor.h>


namespace ns3 {

    class mmWaveInterference : public Object
    {
    public:
        mmWaveInterference ();
        virtual ~mmWaveInterference ();
        static TypeId GetTypeId (void);
        virtual void DoDispose ();
        /**
         * Notify that the PHY is starting a RX attempt
         *
         * @param rxPsd the power spectral density of the signal being RX
         */
        void StartRx (Ptr<const SpectrumValue> rxPsd);
        /**
         * Notify that the RX attempt has ended. 
         */
	void EndRx ();
	/**
         * Notify that a new signal is being perceived in the medium. This
         * method is to be called for all incoming signal, regardless of
         * whether they're useful signals or interferers.
         *
         * @param spd the power spectral density of the new signal
         * @param duration the duration of the new signal
         */
        void AddSignal (Ptr<const SpectrumValue> spd, const Time duration);
        /**
         * Set the Noise Power Spectral Density
         *
         * @param noisePsd the Noise Power Spectral Density in power units
         * (Watt, Pascal...) per Hz.
         */
	void SetNoisePowerSpectralDensity (Ptr<const SpectrumValue> noisePsd);
        void AddPowerChunkProcessor (Ptr<MmWaveChunkProcessor> p);
        void AddSinrChunkProcessor (Ptr<MmWaveChunkProcessor> p);

    private:
        /**
         * Evaluate a Chunk, depending on the Rx status and the last update time
         */
	void ConditionallyEvaluateChunk ();
        /**
         * Adds a signal perceived in the medium.
         * @param spd the power spectral density of the new signal
         */
	void DoAddSignal (Ptr<const SpectrumValue> spd);
        /**
         * Removes a signal perceived in the medium.
         * 
	 * @param spd the power spectral density of the new signal
	 * @param signalId the id of the new signal
         */
	void DoSubtractSignal  (Ptr<const SpectrumValue> spd, uint32_t signalId);
        
	std::list<Ptr<MmWaveChunkProcessor> > m_PowerChunkProcessorList;
        std::list<Ptr<MmWaveChunkProcessor> > m_sinrChunkProcessorList;
    
        bool m_receiving;  //!< True if in Rx status
        /**
         * Stores the power spectral density of the signal whose RX is being attempted
         */
        Ptr<SpectrumValue> m_rxSignal;
        /**
         * Stores the spectral power density of the sum of incoming signals;
         * does not include noise, includes the SPD of the signal being RX
         */
	Ptr<SpectrumValue> m_allSignals;
        Ptr<const SpectrumValue> m_noise;  //!< Noise spectral power density

        Time m_lastChangeTime;  //!< the time of the last change in m_TotalPower

        uint32_t m_lastSignalId;
        uint32_t m_lastSignalIdBeforeReset;
    };

} // namespace ns3

#endif /* MMWAVE_INTERFERENCE_H */
