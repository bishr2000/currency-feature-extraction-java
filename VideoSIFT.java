/**
 * Copyright (c) 2011, The University of Southampton and the individual contributors.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   * 	Redistributions of source code must retain the above copyright notice,
 * 	this list of conditions and the following disclaimer.
 *
 *   *	Redistributions in binary form must reproduce the above copyright notice,
 * 	this list of conditions and the following disclaimer in the documentation
 * 	and/or other materials provided with the distribution.
 *
 *   *	Neither the name of the University of Southampton nor the names of its
 * 	contributors may be used to endorse or promote products derived from this
 * 	software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.openimaj.demos.video;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.*;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;



import org.openimaj.demos.Demo;
import org.openimaj.demos.video.utils.PolygonDrawingListener;
import org.openimaj.demos.video.utils.PolygonExtractionProcessor;
import org.openimaj.feature.local.list.LocalFeatureList;
import org.openimaj.feature.local.matcher.FastBasicKeypointMatcher;
import org.openimaj.feature.local.matcher.MatchingUtilities;
import org.openimaj.feature.local.matcher.consistent.ConsistentLocalFeatureMatcher2d;
import org.openimaj.image.DisplayUtilities.ImageComponent;
import org.openimaj.image.FImage;
import org.openimaj.image.ImageUtilities;
import org.openimaj.image.MBFImage;
import org.openimaj.image.colour.RGBColour;
import org.openimaj.image.colour.Transforms;
import org.openimaj.image.feature.local.engine.DoGSIFTEngine;
import org.openimaj.image.feature.local.engine.asift.ASIFTEngine;
import org.openimaj.image.feature.local.keypoints.Keypoint;
import org.openimaj.image.processing.transform.MBFProjectionProcessor;
import org.openimaj.image.renderer.MBFImageRenderer;
import org.openimaj.math.geometry.shape.Polygon;
import org.openimaj.math.geometry.shape.Rectangle;
import org.openimaj.math.geometry.transforms.HomographyModel;
import org.openimaj.math.geometry.transforms.HomographyRefinement;
import org.openimaj.math.geometry.transforms.MatrixTransformProvider;
import org.openimaj.math.geometry.transforms.TransformUtilities;
import org.openimaj.math.geometry.transforms.check.TransformMatrixConditionCheck;
import org.openimaj.math.geometry.transforms.estimation.RobustHomographyEstimator;
import org.openimaj.math.model.fit.RANSAC;
import org.openimaj.video.Video;
import org.openimaj.video.VideoDisplay;
import org.openimaj.video.VideoDisplayListener;
import org.openimaj.video.capture.Device;
import org.openimaj.video.capture.VideoCapture;
import org.openimaj.video.xuggle.XuggleVideo;

import Jama.Matrix;

import com.sun.speech.freetts.Voice;
import com.sun.speech.freetts.VoiceManager;

/**
 * OpenIMAJ Real-time (ish) SIFT tracking and matching demo
 * 
 * @author Jonathon Hare (jsh2@ecs.soton.ac.uk)
 * @author Sina Samangooei (ss@ecs.soton.ac.uk)
 */

@Demo(
		author = "Jonathon Hare and Sina Samangooei",
		description = "Realtime-ish SIFT-based tracking demonstration." +
				"Hold an object in front of the camera, and press space. Select" +
				"the outline of the object by clicking points on the frozen video " +
				"image, and press C when you're done. Press space to start the video " +
				"again, and the object should be tracked. This demo uses a homography " +
				"to constrain the matches.",
		keywords = { "video", "sift", "object tracking" },
		title = "VideoSIFT")

public class VideoSIFT implements KeyListener, VideoDisplayListener<MBFImage> {
	enum RenderMode {
		SQUARE {
			@Override
			public void render(final MBFImageRenderer renderer, final Matrix transform, final Rectangle rectangle) {
				renderer.drawShape(rectangle.transform(transform), 3, RGBColour.BLUE);
			}
		},
		PICTURE {
			MBFImage toRender = null;
			private Matrix renderToBounds;

			@Override
			public void render(final MBFImageRenderer renderer, final Matrix transform, final Rectangle rectangle) {
				if (this.toRender == null) {
					try {
						this.toRender = ImageUtilities.readMBF(VideoSIFT.class
								.getResource("/org/openimaj/demos/OpenIMAJ.png"));
					} catch (final IOException e) {
						System.err.println("Can't load image to render");
					}
					this.renderToBounds = TransformUtilities.makeTransform(this.toRender.getBounds(), rectangle);
				}

				final MBFProjectionProcessor mbfPP = new MBFProjectionProcessor();
				mbfPP.setMatrix(transform.times(this.renderToBounds));
				mbfPP.accumulate(this.toRender);
				mbfPP.performProjection(0, 0, renderer.getImage());

			}
		},
		VIDEO {
			private XuggleVideo toRender;
			private Matrix renderToBounds;

			@Override
			public void render(final MBFImageRenderer renderer, final Matrix transform, final Rectangle rectangle) {
				if (this.toRender == null) {
					this.toRender = new XuggleVideo(
							VideoSIFT.class.getResource("/org/openimaj/demos/video/keyboardcat.flv"), true);
					this.renderToBounds = TransformUtilities.makeTransform(new Rectangle(0, 0, this.toRender.getWidth(),
							this.toRender.getHeight()), rectangle);
				}

				final MBFProjectionProcessor mbfPP = new MBFProjectionProcessor();
				mbfPP.setMatrix(transform.times(this.renderToBounds));
				mbfPP.accumulate(this.toRender.getNextFrame());
				mbfPP.performProjection(0, 0, renderer.getImage());
			}
		};
		public abstract void render(MBFImageRenderer renderer, Matrix transform, Rectangle rectangle);
	}

	private final VideoCapture capture;
	private final VideoDisplay<MBFImage> videoFrame;
	private final ImageComponent modelFrame;
	private final ImageComponent matchFrame;

	private MBFImage modelImage;

	private MBFImage[] model;

	private MBFImage matches;
	private ConsistentLocalFeatureMatcher2d<Keypoint> matcher;
	private final DoGSIFTEngine engine;
	private final PolygonDrawingListener polygonListener;
	private final JPanel vidPanel;
	//private final JPanel modelPanel;
	private final JPanel matchPanel;
	private RenderMode renderMode = RenderMode.SQUARE;
	private MBFImage currentFrame;
	private static List<Device> videoDevices;
	private LocalFeatureList[] features;
	private Voice voice;
	/**
	 * Construct the demo
	 * 
	 * @param window
	 * @throws Exception
	 */
	public VideoSIFT(final JComponent window) throws Exception {
		this(window, new VideoCapture(224 , 224, videoDevices.get(0)));
	}

	/**
	 * Construct the demo
	 * 
	 * @param window
	 * @param capture
	 * @throws Exception
	 */
	public VideoSIFT(final JComponent window, final VideoCapture capture) throws Exception {
		// read image file stream
		final String oneh = "/org/openimaj/demos/image/100.png";
		final String twoh = "/org/openimaj/demos/image/200.png";
		final String five = "/org/openimaj/demos/image/500.png";
		final String one = "/org/openimaj/demos/image/one.png";
		final String two = "/org/openimaj/demos/image/2000.png";
		final String fivet = "/org/openimaj/demos/image/5000.png";

		this.model = new MBFImage[]{ImageUtilities.readMBF(VideoSIFT.class.getResourceAsStream(oneh)),
									ImageUtilities.readMBF(VideoSIFT.class.getResourceAsStream(twoh)),
									ImageUtilities.readMBF(VideoSIFT.class.getResourceAsStream(five)),
									ImageUtilities.readMBF(VideoSIFT.class.getResourceAsStream(one)),
									ImageUtilities.readMBF(VideoSIFT.class.getResourceAsStream(two)),
									ImageUtilities.readMBF(VideoSIFT.class.getResourceAsStream(fivet))};


		//the default image model for the engine database
		//this.modelImage = model[1];

		final int width = capture.getWidth();
		final int height = capture.getHeight();

		this.capture = capture;
		this.polygonListener = new PolygonDrawingListener();

		GridBagConstraints gbc = new GridBagConstraints();

		final JLabel label = new JLabel("<html><body><p>then press c to load the features.</p></body></html>");

		gbc.gridx = 0;
		gbc.gridy = 0;
		gbc.gridwidth = 2;
		gbc.insets = new Insets(8, 8, 8, 8);
		window.add(label, gbc);

		this.vidPanel = new JPanel(new GridBagLayout());
		this.vidPanel.setBorder(BorderFactory.createTitledBorder("Live Video"));
		this.videoFrame = VideoDisplay.createVideoDisplay(capture, this.vidPanel);
//		gbc = new GridBagConstraints();
//		gbc.gridy = 1;
//		gbc.gridx = 0;
//		gbc.gridwidth = 1;
		window.add(this.vidPanel, gbc);

		this.modelFrame = new ImageComponent(true, false);
		this.modelFrame.setShowPixelColours(false);
		this.modelFrame.setShowXYPosition(false);
		this.modelFrame.removeMouseListener(this.modelFrame);
		this.modelFrame.removeMouseMotionListener(this.modelFrame);
		this.modelFrame.setSize(width, height);
		this.modelFrame.setPreferredSize(new Dimension(width, height));
		this.matchPanel = new JPanel(new GridBagLayout());
		this.matchPanel.setBorder(BorderFactory.createTitledBorder("Matches"));
		this.matchFrame = new ImageComponent(true, false);
		this.matchFrame.setShowPixelColours(false);
		this.matchFrame.setShowXYPosition(false);
		this.matchFrame.removeMouseListener(this.matchFrame);
		this.matchFrame.removeMouseMotionListener(this.matchFrame);
		this.matchFrame.setSize(width * 2, height);
		this.matchFrame.setPreferredSize(new Dimension(width *3, height));
		this.matchPanel.add(this.matchFrame);
//		gbc = new GridBagConstraints();
//		gbc.anchor = GridBagConstraints.PAGE_END;
//		gbc.gridy = 2;
//		gbc.gridx = 0;
//		gbc.gridwidth = 2;
//		window.add(this.matchPanel, gbc);

		final JFrame window2 = new JFrame();
		window2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		window2.setResizable(true);
		window2.setExtendedState(JFrame.MAXIMIZED_BOTH);
		window2.setLayout(new GridBagLayout());

		window2.add(this.matchPanel);
		window2.pack();
		window2.setVisible(true);

		this.videoFrame.getScreen().addMouseListener(this.polygonListener);

		this.videoFrame.addVideoListener(this);
		this.engine = new DoGSIFTEngine();
		this.engine.getOptions().setDoubleInitialImage(false);

	}

	@Override
	public synchronized void keyPressed(final KeyEvent key) {
		System.setProperty("freetts.voices", "com.sun.speech.freetts.en.us.cmu_us_kal.KevinVoiceDirectory");
		this.voice = VoiceManager.getInstance().getVoice("kevin16");

		if (key.getKeyCode() == KeyEvent.VK_SPACE) {
			this.videoFrame.togglePause();
		} else if (key.getKeyChar() == 'c') {

			try {
				if (this.matcher == null) {
					final RobustHomographyEstimator ransac = new RobustHomographyEstimator(0.5, 1500,
							new RANSAC.PercentageInliersStoppingCondition(0.6), HomographyRefinement.NONE,
							new TransformMatrixConditionCheck<HomographyModel>(10000));

					this.matcher = new ConsistentLocalFeatureMatcher2d<Keypoint>(
							new FastBasicKeypointMatcher<Keypoint>(8));

					this.matcher.setFittingModel(ransac);

				}

			} catch (final Exception e) {
				e.printStackTrace();
			}

			this.modelImage = this.model[0];
			this.modelFrame.setImage(ImageUtilities.createBufferedImageForDisplay(this.modelImage));
		}

		this.features = new LocalFeatureList[]{
				engine.findFeatures(Transforms.calculateIntensityNTSC(this.model[0])),
				engine.findFeatures(Transforms.calculateIntensityNTSC(this.model[1])),
				engine.findFeatures(Transforms.calculateIntensityNTSC(this.model[2])),
				engine.findFeatures(Transforms.calculateIntensityNTSC(this.model[3])),
				engine.findFeatures(Transforms.calculateIntensityNTSC(this.model[4])),
				engine.findFeatures(Transforms.calculateIntensityNTSC(this.model[5]))};

	}

	@Override
	public void keyReleased(final KeyEvent arg0) {
	}

	@Override
	public void keyTyped(final KeyEvent arg0) {
	}

	@Override
	public synchronized void afterUpdate(final VideoDisplay<MBFImage> display) {
		voice.allocate();
		if (this.matcher != null && !this.videoFrame.isPaused()) {

			final MBFImage capImg = this.currentFrame;

			final FImage intenseImage = Transforms.calculateIntensityNTSC(capImg);


			// detect keypoints in an image "capImg" using the engine and return a list of local features
			final LocalFeatureList<Keypoint> kpl = this.engine.findFeatures(intenseImage);

			// create a renderer to draw points on an image
			final MBFImageRenderer renderer = capImg.createRenderer();
			renderer.drawPoints(kpl, RGBColour.RED, 3);

			// image of matches to locate the keypoints on

			for(int i=0; i<6; i++) {

				this.matcher.setModelFeatures(features[i]);
				// Attempt to find matches between the model features from the database, and given query features.
				if (this.matcher.findMatches(kpl)
						&& ((MatrixTransformProvider) this.matcher.getModel()).getTransform().cond() < 1e6) {

					try {
						final Matrix boundsToPoly = ((MatrixTransformProvider) this.matcher.getModel()).getTransform()
								.inverse();

						if (modelImage.getBounds().transform(boundsToPoly).isConvex()) {
							this.renderMode.render(renderer, boundsToPoly, this.modelImage.getBounds());
							if(voice != null){


								switch(i){
									case 0:
										System.out.println("it's 100");
										voice.speak("one hundred");
										break;
									case 1:
										System.out.println("it's 200");
										voice.speak("two hundred");
										break;
									case 2:
										System.out.println("it's 500");
										voice.speak("five hundred");
										break;
									case 3:
										System.out.println("it's 1000");
										voice.speak("one thousand");
										break;
									case 4:
										System.out.println("it's 2000");
										voice.speak("two thousand");
										break;

									case 5:
										System.out.println("it's 5000");
										voice.speak("five thousand");
										break;
									default:
										System.out.println("none");
								}

							}else{
								System.out.println("error");
							}
							matches = MatchingUtilities
									.drawMatches(this.model[i], capImg, this.matcher.getMatches(), RGBColour.GREEN);
							break;
						}
					} catch (final RuntimeException e) {
						System.out.println("something went wrong: " + e);
					}


				} else {
					matches = MatchingUtilities
							.drawMatches(this.model[i], capImg, this.matcher.getMatches(), RGBColour.BLUE);
				}

				voice.deallocate();
			}
			this.matchPanel.setPreferredSize(this.matchPanel.getSize());
			this.matchFrame.setImage(ImageUtilities.createBufferedImageForDisplay(matches));
		}


	}

	@Override
	public void beforeUpdate(final MBFImage frame) {
		if (!this.videoFrame.isPaused())
			this.currentFrame = frame.clone();
		else {
			frame.drawImage(currentFrame, 0, 0);
		}
		this.polygonListener.drawPoints(frame);
	}

	/**
	 * Stop capture
	 */
	public void stop() {
		this.videoFrame.close();
		this.capture.stopCapture();
	}

	/**
	 * Main method
	 * 
	 * @param args
	 *            ignored
	 * @throws Exception
	 */
	public static void main(final String[] args) throws Exception {
		videoDevices = VideoCapture.getVideoDevices();
		final JFrame window = new JFrame();
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		window.setResizable(true);
		window.setExtendedState(JFrame.MAXIMIZED_BOTH);
		window.setLayout(new GridBagLayout());
		final JPanel c = new JPanel();
		c.setLayout(new GridBagLayout());
		window.getContentPane().add(c);
		final VideoSIFT vs = new VideoSIFT(c);
		SwingUtilities.getRoot(window).addKeyListener(vs);
		window.pack();
		window.setVisible(true);
	}
}
