
#ifdef NANOGUI_PYTHON

#include "python.h"

/* Developer note: need to make a change to this file?
 * Please raise an Issue on GitHub describing what needs to change.  This file
 * was generated, so the scripts that generated it needs to update as well.
 */

void register_constants_entypo(py::module &m) {
    /* Entypo constants */
    {
        #define C(name) g.attr("ICON_" #name) = py::int_(ENTYPO_ICON_##name);
        py::module g = m.def_submodule("entypo");
        C(500PX);
        C(500PX_WITH_CIRCLE);
        C(ADD_TO_LIST);
        C(ADD_USER);
        C(ADDRESS);
        C(ADJUST);
        C(AIR);
        C(AIRCRAFT);
        C(AIRCRAFT_LANDING);
        C(AIRCRAFT_TAKE_OFF);
        C(ALIGN_BOTTOM);
        C(ALIGN_HORIZONTAL_MIDDLE);
        C(ALIGN_LEFT);
        C(ALIGN_RIGHT);
        C(ALIGN_TOP);
        C(ALIGN_VERTICAL_MIDDLE);
        C(APP_STORE);
        C(ARCHIVE);
        C(AREA_GRAPH);
        C(ARROW_BOLD_DOWN);
        C(ARROW_BOLD_LEFT);
        C(ARROW_BOLD_RIGHT);
        C(ARROW_BOLD_UP);
        C(ARROW_DOWN);
        C(ARROW_LEFT);
        C(ARROW_LONG_DOWN);
        C(ARROW_LONG_LEFT);
        C(ARROW_LONG_RIGHT);
        C(ARROW_LONG_UP);
        C(ARROW_RIGHT);
        C(ARROW_UP);
        C(ARROW_WITH_CIRCLE_DOWN);
        C(ARROW_WITH_CIRCLE_LEFT);
        C(ARROW_WITH_CIRCLE_RIGHT);
        C(ARROW_WITH_CIRCLE_UP);
        C(ATTACHMENT);
        C(AWARENESS_RIBBON);
        C(BACK);
        C(BACK_IN_TIME);
        C(BAIDU);
        C(BAR_GRAPH);
        C(BASECAMP);
        C(BATTERY);
        C(BEAMED_NOTE);
        C(BEHANCE);
        C(BELL);
        C(BLACKBOARD);
        C(BLOCK);
        C(BOOK);
        C(BOOKMARK);
        C(BOOKMARKS);
        C(BOWL);
        C(BOX);
        C(BRIEFCASE);
        C(BROWSER);
        C(BRUSH);
        C(BUCKET);
        C(BUG);
        C(CAKE);
        C(CALCULATOR);
        C(CALENDAR);
        C(CAMERA);
        C(CCW);
        C(CHAT);
        C(CHECK);
        C(CHEVRON_DOWN);
        C(CHEVRON_LEFT);
        C(CHEVRON_RIGHT);
        C(CHEVRON_SMALL_DOWN);
        C(CHEVRON_SMALL_LEFT);
        C(CHEVRON_SMALL_RIGHT);
        C(CHEVRON_SMALL_UP);
        C(CHEVRON_THIN_DOWN);
        C(CHEVRON_THIN_LEFT);
        C(CHEVRON_THIN_RIGHT);
        C(CHEVRON_THIN_UP);
        C(CHEVRON_UP);
        C(CHEVRON_WITH_CIRCLE_DOWN);
        C(CHEVRON_WITH_CIRCLE_LEFT);
        C(CHEVRON_WITH_CIRCLE_RIGHT);
        C(CHEVRON_WITH_CIRCLE_UP);
        C(CIRCLE);
        C(CIRCLE_WITH_CROSS);
        C(CIRCLE_WITH_MINUS);
        C(CIRCLE_WITH_PLUS);
        C(CIRCULAR_GRAPH);
        C(CLAPPERBOARD);
        C(CLASSIC_COMPUTER);
        C(CLIPBOARD);
        C(CLOCK);
        C(CLOUD);
        C(CODE);
        C(COG);
        C(COLOURS);
        C(COMPASS);
        C(CONTROLLER_FAST_BACKWARD);
        C(CONTROLLER_FAST_FORWARD);
        C(CONTROLLER_JUMP_TO_START);
        C(CONTROLLER_NEXT);
        C(CONTROLLER_PAUS);
        C(CONTROLLER_PLAY);
        C(CONTROLLER_RECORD);
        C(CONTROLLER_STOP);
        C(CONTROLLER_VOLUME);
        C(COPY);
        C(CREATIVE_CLOUD);
        C(CREATIVE_COMMONS);
        C(CREATIVE_COMMONS_ATTRIBUTION);
        C(CREATIVE_COMMONS_NODERIVS);
        C(CREATIVE_COMMONS_NONCOMMERCIAL_EU);
        C(CREATIVE_COMMONS_NONCOMMERCIAL_US);
        C(CREATIVE_COMMONS_PUBLIC_DOMAIN);
        C(CREATIVE_COMMONS_REMIX);
        C(CREATIVE_COMMONS_SHARE);
        C(CREATIVE_COMMONS_SHAREALIKE);
        C(CREDIT);
        C(CREDIT_CARD);
        C(CROP);
        C(CROSS);
        C(CUP);
        C(CW);
        C(CYCLE);
        C(DATABASE);
        C(DIAL_PAD);
        C(DIRECTION);
        C(DOCUMENT);
        C(DOCUMENT_LANDSCAPE);
        C(DOCUMENTS);
        C(DOT_SINGLE);
        C(DOTS_THREE_HORIZONTAL);
        C(DOTS_THREE_VERTICAL);
        C(DOTS_TWO_HORIZONTAL);
        C(DOTS_TWO_VERTICAL);
        C(DOWNLOAD);
        C(DRIBBBLE);
        C(DRIBBBLE_WITH_CIRCLE);
        C(DRINK);
        C(DRIVE);
        C(DROP);
        C(DROPBOX);
        C(EDIT);
        C(EMAIL);
        C(EMOJI_FLIRT);
        C(EMOJI_HAPPY);
        C(EMOJI_NEUTRAL);
        C(EMOJI_SAD);
        C(ERASE);
        C(ERASER);
        C(EVERNOTE);
        C(EXPORT);
        C(EYE);
        C(EYE_WITH_LINE);
        C(FACEBOOK);
        C(FACEBOOK_WITH_CIRCLE);
        C(FEATHER);
        C(FINGERPRINT);
        C(FLAG);
        C(FLASH);
        C(FLASHLIGHT);
        C(FLAT_BRUSH);
        C(FLATTR);
        C(FLICKR);
        C(FLICKR_WITH_CIRCLE);
        C(FLOW_BRANCH);
        C(FLOW_CASCADE);
        C(FLOW_LINE);
        C(FLOW_PARALLEL);
        C(FLOW_TREE);
        C(FLOWER);
        C(FOLDER);
        C(FOLDER_IMAGES);
        C(FOLDER_MUSIC);
        C(FOLDER_VIDEO);
        C(FORWARD);
        C(FOURSQUARE);
        C(FUNNEL);
        C(GAME_CONTROLLER);
        C(GAUGE);
        C(GITHUB);
        C(GITHUB_WITH_CIRCLE);
        C(GLOBE);
        C(GOOGLE_DRIVE);
        C(GOOGLE_HANGOUTS);
        C(GOOGLE_PLAY);
        C(GOOGLE_PLUS);
        C(GOOGLE_PLUS_WITH_CIRCLE);
        C(GRADUATION_CAP);
        C(GRID);
        C(GROOVESHARK);
        C(HAIR_CROSS);
        C(HAND);
        C(HEART);
        C(HEART_OUTLINED);
        C(HELP);
        C(HELP_WITH_CIRCLE);
        C(HOME);
        C(HOUR_GLASS);
        C(HOUZZ);
        C(ICLOUD);
        C(IMAGE);
        C(IMAGE_INVERTED);
        C(IMAGES);
        C(INBOX);
        C(INFINITY);
        C(INFO);
        C(INFO_WITH_CIRCLE);
        C(INSTAGRAM);
        C(INSTAGRAM_WITH_CIRCLE);
        C(INSTALL);
        C(KEY);
        C(KEYBOARD);
        C(LAB_FLASK);
        C(LANDLINE);
        C(LANGUAGE);
        C(LAPTOP);
        C(LASTFM);
        C(LASTFM_WITH_CIRCLE);
        C(LAYERS);
        C(LEAF);
        C(LEVEL_DOWN);
        C(LEVEL_UP);
        C(LIFEBUOY);
        C(LIGHT_BULB);
        C(LIGHT_DOWN);
        C(LIGHT_UP);
        C(LINE_GRAPH);
        C(LINK);
        C(LINKEDIN);
        C(LINKEDIN_WITH_CIRCLE);
        C(LIST);
        C(LOCATION);
        C(LOCATION_PIN);
        C(LOCK);
        C(LOCK_OPEN);
        C(LOG_OUT);
        C(LOGIN);
        C(LOOP);
        C(MAGNET);
        C(MAGNIFYING_GLASS);
        C(MAIL);
        C(MAIL_WITH_CIRCLE);
        C(MAN);
        C(MAP);
        C(MASK);
        C(MEDAL);
        C(MEDIUM);
        C(MEDIUM_WITH_CIRCLE);
        C(MEGAPHONE);
        C(MENU);
        C(MERGE);
        C(MESSAGE);
        C(MIC);
        C(MINUS);
        C(MIXI);
        C(MOBILE);
        C(MODERN_MIC);
        C(MOON);
        C(MOUSE);
        C(MOUSE_POINTER);
        C(MUSIC);
        C(NETWORK);
        C(NEW);
        C(NEW_MESSAGE);
        C(NEWS);
        C(NEWSLETTER);
        C(NOTE);
        C(NOTIFICATION);
        C(NOTIFICATIONS_OFF);
        C(OLD_MOBILE);
        C(OLD_PHONE);
        C(ONEDRIVE);
        C(OPEN_BOOK);
        C(PALETTE);
        C(PAPER_PLANE);
        C(PAYPAL);
        C(PENCIL);
        C(PHONE);
        C(PICASA);
        C(PIE_CHART);
        C(PIN);
        C(PINTEREST);
        C(PINTEREST_WITH_CIRCLE);
        C(PLUS);
        C(POPUP);
        C(POWER_PLUG);
        C(PRICE_RIBBON);
        C(PRICE_TAG);
        C(PRINT);
        C(PROGRESS_EMPTY);
        C(PROGRESS_FULL);
        C(PROGRESS_ONE);
        C(PROGRESS_TWO);
        C(PUBLISH);
        C(QQ);
        C(QQ_WITH_CIRCLE);
        C(QUOTE);
        C(RADIO);
        C(RAFT);
        C(RAFT_WITH_CIRCLE);
        C(RAINBOW);
        C(RDIO);
        C(RDIO_WITH_CIRCLE);
        C(REMOVE_USER);
        C(RENREN);
        C(REPLY);
        C(REPLY_ALL);
        C(RESIZE_100_PERCENT);
        C(RESIZE_FULL_SCREEN);
        C(RETWEET);
        C(ROCKET);
        C(ROUND_BRUSH);
        C(RSS);
        C(RULER);
        C(SAVE);
        C(SCISSORS);
        C(SCRIBD);
        C(SELECT_ARROWS);
        C(SHARE);
        C(SHARE_ALTERNATIVE);
        C(SHAREABLE);
        C(SHIELD);
        C(SHOP);
        C(SHOPPING_BAG);
        C(SHOPPING_BASKET);
        C(SHOPPING_CART);
        C(SHUFFLE);
        C(SIGNAL);
        C(SINA_WEIBO);
        C(SKYPE);
        C(SKYPE_WITH_CIRCLE);
        C(SLIDESHARE);
        C(SMASHING);
        C(SOUND);
        C(SOUND_MIX);
        C(SOUND_MUTE);
        C(SOUNDCLOUD);
        C(SPORTS_CLUB);
        C(SPOTIFY);
        C(SPOTIFY_WITH_CIRCLE);
        C(SPREADSHEET);
        C(SQUARED_CROSS);
        C(SQUARED_MINUS);
        C(SQUARED_PLUS);
        C(STAR);
        C(STAR_OUTLINED);
        C(STOPWATCH);
        C(STUMBLEUPON);
        C(STUMBLEUPON_WITH_CIRCLE);
        C(SUITCASE);
        C(SWAP);
        C(SWARM);
        C(SWEDEN);
        C(SWITCH);
        C(TABLET);
        C(TABLET_MOBILE_COMBO);
        C(TAG);
        C(TEXT);
        C(TEXT_DOCUMENT);
        C(TEXT_DOCUMENT_INVERTED);
        C(THERMOMETER);
        C(THUMBS_DOWN);
        C(THUMBS_UP);
        C(THUNDER_CLOUD);
        C(TICKET);
        C(TIME_SLOT);
        C(TOOLS);
        C(TRAFFIC_CONE);
        C(TRASH);
        C(TREE);
        C(TRIANGLE_DOWN);
        C(TRIANGLE_LEFT);
        C(TRIANGLE_RIGHT);
        C(TRIANGLE_UP);
        C(TRIPADVISOR);
        C(TROPHY);
        C(TUMBLR);
        C(TUMBLR_WITH_CIRCLE);
        C(TV);
        C(TWITTER);
        C(TWITTER_WITH_CIRCLE);
        C(TYPING);
        C(UNINSTALL);
        C(UNREAD);
        C(UNTAG);
        C(UPLOAD);
        C(UPLOAD_TO_CLOUD);
        C(USER);
        C(USERS);
        C(V_CARD);
        C(VIDEO);
        C(VIDEO_CAMERA);
        C(VIMEO);
        C(VIMEO_WITH_CIRCLE);
        C(VINE);
        C(VINE_WITH_CIRCLE);
        C(VINYL);
        C(VK);
        C(VK_ALTERNITIVE);
        C(VK_WITH_CIRCLE);
        C(VOICEMAIL);
        C(WALLET);
        C(WARNING);
        C(WATER);
        C(WINDOWS_STORE);
        C(XING);
        C(XING_WITH_CIRCLE);
        C(YELP);
        C(YOUKO);
        C(YOUKO_WITH_CIRCLE);
        C(YOUTUBE);
        C(YOUTUBE_WITH_CIRCLE);

        #undef C
    }
}

#endif
