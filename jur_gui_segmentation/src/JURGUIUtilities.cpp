#include "../include/jur_gui_segmentation/JURGUIUtilities.hpp"


JURGUI::JURGUIUtilities::JURGUIUtilities()
{
}


/** Adjusting window in center of active screen */
void JURGUI::JURGUIUtilities::displayInCenter(QWidget* _widget)
{
    if ( !_widget )
        return;

    QDesktopWidget* d = QApplication::desktop();
    QRect d_rect = d->screenGeometry( d->screenNumber( QCursor::pos() ) );

    int d_x = d_rect.width ();
    int d_y = d_rect.height();

    int x = _widget->width ();
    int y = _widget->height();

    _widget->move( d_x / 2 - x / 2 + d_rect.left(), d_y / 2 - y / 2 + d_rect.top() );
}

